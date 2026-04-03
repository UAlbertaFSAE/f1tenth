// NOLINTBEGIN
#include "detection_generator/detection_generator.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace {
std::string trim_copy(const std::string& input) {
  std::size_t start = 0;
  while (start < input.size() && std::isspace(static_cast<unsigned char>(input[start]))) {
    start++;
  }

  std::size_t end = input.size();
  while (end > start && std::isspace(static_cast<unsigned char>(input[end - 1]))) {
    end--;
  }

  return input.substr(start, end - start);
}
}  // namespace

DetectionGenerator::DetectionGenerator() : Node("detection_generator_node"), next_frame_index_(0) {
  this->declare_parameter("track_type", "straight");
  this->declare_parameter("csv_path", "");
  this->declare_parameter("cone_topic", "/cone_data");
  this->declare_parameter("loop_track", true);

  const std::string track_type = this->get_parameter("track_type").as_string();
  const std::string configured_csv_path = this->get_parameter("csv_path").as_string();
  const std::string cone_topic = this->get_parameter("cone_topic").as_string();
  loop_track_ = this->get_parameter("loop_track").as_bool();

  const std::string csv_path = resolve_csv_path(configured_csv_path, track_type);

  frames_ = read_csv(csv_path);
  cone_publisher_ = create_publisher<rc_interfaces::msg::Cones>(cone_topic, rclcpp::QoS(10));

  constexpr double kFrameRateHz = 5.0;
  const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / kFrameRateHz));
  publish_timer_ =
      this->create_wall_timer(period, std::bind(&DetectionGenerator::publish_next_frame, this));

  std::size_t total_cones = 0;
  for (const auto& frame : frames_) {
    total_cones += frame.cones.cones.size();
  }

  RCLCPP_INFO(this->get_logger(), "Loaded %zu frames (%zu cones) from %s", frames_.size(),
              total_cones, csv_path.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing one cone frame at %.2f Hz on %s", kFrameRateHz,
              cone_topic.c_str());
}

void DetectionGenerator::publish_next_frame() {
  if (frames_.empty()) {
    return;
  }

  if (next_frame_index_ >= frames_.size()) {
    if (!loop_track_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Finished publishing all cone frames once");
      return;
    }
    next_frame_index_ = 0;
  }

  const auto& frame = frames_[next_frame_index_];
  cone_publisher_->publish(frame.cones);
  RCLCPP_DEBUG(this->get_logger(), "Published frame_id=%d with %zu cones", frame.frame_id,
               frame.cones.cones.size());
  next_frame_index_++;
}

std::string DetectionGenerator::resolve_csv_path(const std::string& configured_path,
                                                 const std::string& track_type) const {
  if (!configured_path.empty()) {
    return configured_path;
  }

  std::string normalized_track = track_type;
  std::transform(normalized_track.begin(), normalized_track.end(), normalized_track.begin(),
                 [](unsigned char c) { return std::tolower(c); });

  if (normalized_track != "straight" && normalized_track != "eight" && normalized_track != "curved") {
    RCLCPP_WARN(this->get_logger(),
                "Unsupported track_type '%s'. Falling back to 'straight'. Valid values are: "
                "straight, eight, curved.",
                track_type.c_str());
    normalized_track = "straight";
  }

  const std::string share_dir = ament_index_cpp::get_package_share_directory("detection_generator");
  return share_dir + "/data/" + normalized_track + ".csv";
}

std::vector<DetectionGenerator::ConeFrame> DetectionGenerator::read_csv(const std::string& path) {
  std::fstream file;
  file.open(path);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open track csv: %s", path.c_str());
    return {};
  }

  RCLCPP_INFO(this->get_logger(), "Opened track csv: %s", path.c_str());

  std::string line;
  std::vector<ConeFrame> frames;
  std::unordered_map<int, std::size_t> frame_index_by_id;

  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }

    std::stringstream s(line);
    std::vector<std::string> fields;
    std::string token;
    while (std::getline(s, token, ',')) {
      fields.push_back(token);
    }

    if (fields.size() != kCsvFieldCount) {
      RCLCPP_WARN(this->get_logger(),
                  "Skipping malformed row in %s. Expected %zu fields, got %zu: %s", path.c_str(),
                  kCsvFieldCount, fields.size(), line.c_str());
      continue;
    }

    if (fields[0].empty()) {
      continue;
    }

    // Skip header row.
    if (!std::isdigit(static_cast<unsigned char>(fields[0][0])) && fields[0][0] != '-') {
      continue;
    }

    try {
      const int frame_id = std::stoi(trim_copy(fields[0]));

      rc_interfaces::msg::Cone cone;
      cone.x = std::stof(trim_copy(fields[1]));
      cone.y = std::stof(trim_copy(fields[2]));
      cone.color = trim_copy(fields[3]);

      if (cone.color.empty()) {
        RCLCPP_WARN(this->get_logger(), "Skipping row with empty cone color in %s: %s",
                    path.c_str(), line.c_str());
        continue;
      }

      auto frame_it = frame_index_by_id.find(frame_id);
      if (frame_it == frame_index_by_id.end()) {
        ConeFrame new_frame;
        new_frame.frame_id = frame_id;
        frames.push_back(new_frame);
        frame_index_by_id[frame_id] = frames.size() - 1;
        frame_it = frame_index_by_id.find(frame_id);
      }

      frames[frame_it->second].cones.cones.push_back(cone);
    } catch (const std::invalid_argument&) {
      RCLCPP_WARN(this->get_logger(), "Skipping non-numeric row in %s: %s", path.c_str(),
                  line.c_str());
    } catch (const std::out_of_range&) {
      RCLCPP_WARN(this->get_logger(), "Skipping out-of-range row in %s: %s", path.c_str(),
                  line.c_str());
    }
  }

  file.close();
  return frames;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DetectionGenerator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
// NOLINTEND
