#include "image_generator/image_generator.hpp"

ImageGenerator::ImageGenerator() : Node("image_generator") {
  fps = this->declare_parameter("fps", 30);
  image_topic = this->declare_parameter("image_topic", "/image_generator/image");
  dataset_path = this->declare_parameter(
      "dataset_path", "/f1tenth/src/perception/sample_data/images");

  image_publisher = this->create_publisher<sensor_msgs::msg::Image>(image_topic, 10);

  std::chrono::milliseconds ms(1000 / fps);
  timer = this->create_wall_timer(ms, std::bind(&ImageGenerator::timer_callback, this));

  // build vector of image_paths only once at the start
  for (const auto& entry : fs::directory_iterator(dataset_path)) {
    if (entry.is_regular_file()) {
      image_paths.push_back(entry.path());
    }
  }
}

// check if I should publish empty messages on errors or just return and not publish anything
void ImageGenerator::timer_callback() {
  if (image_paths.empty()) {
    std::cerr << "No files found in the directory." << std::endl;
    return;
  }

  // randomly select an image
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, image_paths.size() - 1);
  fs::path selected_file = image_paths[dis(gen)];

  // read the selected file using OpenCV
  cv::Mat image = cv::imread(selected_file.string(), cv::IMREAD_COLOR);
  if (image.empty()) {
    std::cerr << "Could not open or find the image: " << selected_file << std::endl;
    return;
  }

  cv::Size sz = image.size();
  RCLCPP_INFO(this->get_logger(), "Image Width: %d", sz.width);
  RCLCPP_INFO(this->get_logger(), "Image Height: %d", sz.height);

  img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
  image_publisher->publish(*img_msg.get());
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageGenerator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
