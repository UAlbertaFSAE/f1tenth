#include <memory>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "../include/CanController.hpp"
#include "../include/SerialHandler.hpp"
/*
  1. Create the data frame that the telemetry node is going to receive/transmit.
  So the telemetry node is also going to publish this data frame

    - CANBUS messages are going to be sent to a seperate process, and that process needs
    to communicate with the telemetry node. And the telemetry node needs to receive it and send
    information

    publishers for multiple topics
    subscribers only have one topic
    only one subscriber on a topic -> and my node is going to multiplex it to know
    where it should send the info to
  2.
*/

#define MAX_TOPICS 5

using namespace std;

CanController::CanController(string topics_loc): Node("CanController"){
  createPublishersByTopics(
    readFromLoc(topics_loc)
    );
}

vector<string> CanController::readFromLoc(string loc){
  vector<string> out;

  return out;
}

/*
  we can assign a standard callback, that is just going to post a standard message to all
  the topics
 */
void CanController::createPublishersByTopics(vector<string> topics_dict) {
  for (int i = 0; i < topicsList_.size(); ++i) {
    std::unique_ptr<TelemetryPublisher> telemetry_publisher = std::make_unique<TelemetryPublisher>();

    telemetry_publisher->topic = topicsList_.at(i);
    telemetry_publisher->publisher =
      this->create_publisher<std_msgs::msg::String>(telemetry_publisher->topic, 0);

    if (publisherTable_->head != nullptr) {
      publisherTable_->head = telemetry_publisher;
    } else {
      auto curr = publisherTable_->head;

      while (curr != nullptr)
        curr = curr->next;
      curr->next = telemetry_publisher;

      publisherTable_->count++;
    }
  }
}

int main(int argc, char* argv[]) {
  // init rclcpp
}