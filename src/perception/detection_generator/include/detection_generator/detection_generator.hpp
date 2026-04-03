// NOLINTBEGIN
#include <fstream>
#include <string>
#include <vector>

#include "rc_interfaces/msg/cones.hpp"
#include "rclcpp/rclcpp.hpp"

class DetectionGenerator : public rclcpp::Node {
 public:
  DetectionGenerator();

 private:
  struct ConeFrame {
    int frame_id;
    rc_interfaces::msg::Cones cones;
  };

  static constexpr std::size_t kCsvFieldCount = 4;

  std::size_t next_frame_index_;
  bool loop_track_;
  void publish_next_frame();
  std::vector<ConeFrame> read_csv(const std::string& path);
  std::string resolve_csv_path(const std::string& configured_path,
                               const std::string& track_type) const;
  std::vector<ConeFrame> frames_;
  rclcpp::Publisher<rc_interfaces::msg::Cones>::SharedPtr cone_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};
// NOLINTEND
