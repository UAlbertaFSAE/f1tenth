#include <memory>
#include <rclcpp/publisher.hpp>
#include "std_msgs/msg/string.hpp"
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "../include/TelemetryNode.hpp"
#include <chrono>

/*
  1. Missing the creation of the custom message protocol for telemetry, sends
  simple messages for now
 */

using namespace std;
using namespace std::chrono_literals;

TelemetryNode::TelemetryNode(const vector<Topic>* topics): Node("TelemetryNode"){

  for (size_t i = 0; i < topics->size(); ++i) {
    publishers_.push_back(
      create_topic_publisher(topics->at(i))
    );
  }
}

/**
  Creating a publisher for the topic. Topic is a structure, but can later become
  a class if each topic has a unique callback. For now, every topic has a standard timer
  callback
**/
TelemetryPublisher TelemetryNode::create_topic_publisher(Topic topic) {
  TelemetryPublisher publisher;
  publisher.topic = topic;
  publisher.publisher = this->create_publisher<std_msgs::msg::String>(topic.name, 0);

  publisher.timer = this->create_wall_timer(10000ms, [this, publisher] {
    // temporary code until the message type is created
    RCLCPP_INFO(this->get_logger(), "A telemetry publisher is attempting to call its callback...");

    Topic topic = publisher.topic;
    std::queue<std::string>* message_queue = &topic.message_queue;

    if (!message_queue->empty()) {
      std_msgs::msg::String message;
      message.data = message_queue->front();
      message_queue->pop();

      publisher.publisher->publish(message);
    }
  });

  return publisher;
}

void create_telemetry_subscriber() {
  
}