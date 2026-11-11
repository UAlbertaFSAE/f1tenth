#include <memory>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "../include/CanController.hpp"


/*
  1. Missing the creation of the custom message protocol for telemetry, sends
  simple messages for now
 */

using namespace std;

CanController::CanController(vector<Topic> topics): Node("CanController"){
  for (int i = 0; i < topics.size(); ++i) {
    publishers_.push_back(
      create_topic_publisher(topics[i])
    );
  }
}

vector<string> CanController::readFromLoc(string loc){
  vector<string> out;

  return out;
}

/**
  Creating a publisher for the topic. Topic is a structure, but can later become
  a class if each topic has a unique callback. For now, every topic has a standard timer
  callback
**/
TelemetryPublisher CanController::create_topic_publisher(Topic topic) {
  TelemetryPublisher publisher;
  publisher.topic = topic;
  publisher.publisher = this->create_publisher<std_msgs::msg::String>(topic.name, 0);

  rclcpp::TimerBase::SharedPtr timer_ = this->create_wall_timer(1000ms, [this, publisher] {
    // temporary code until the message type is created
    Topic topic = publisher.topic;
    std::queue <std::string> message_queue = topic.message_queue;

    if (!message_queue.empty()) {
      std_msgs::msg::String message;
      message.data = message_queue.front();
      message_queue.pop();

      publisher.publisher->publish(message);
    }

  });
}

int main(int argc, char* argv[]) {
  /*
    Topics are structs that will include more information about the specific topic
    for the telemetry node. initializing the vector array of topics and pass it to
    CanController
   */
}