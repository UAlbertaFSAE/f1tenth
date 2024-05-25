#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <random>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

using std::placeholders::_1;
namespace fs = std::filesystem;
using namespace std::chrono_literals;

class ImageGenerator : public rclcpp::Node {
 public:
  ImageGenerator();

 private:
  // image generation callback, publishes sampled image from dataset
  void timer_callback();

  int fps;
  std::string dataset_path;
  std::string image_topic;
  std::vector<fs::path> image_paths;
  sensor_msgs::msg::Image::SharedPtr img_msg;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
  rclcpp::TimerBase::SharedPtr timer;
};
