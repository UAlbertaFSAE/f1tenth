#ifndef CANCONTROLLER_HPP
#define CANCONTROLLER_HPP
#include <list>
#include <memory>
#include <string>
#include <vector>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::string;
using std::list;
using std::vector;


/**
  The SerialReceiver is going to receive a message, and then loop through a linkedlist
  of the TelemetryPublisher structs until it finds one that is meant for the message.

  Once the TelemetryPublisher receives the message, it publishes the message to the correct topic.

  So, TelemetryPublisher needs a way to
 **/

using Topic = struct Topic {
  std::string name;
  string message_type;
  std::queue<std::string> message_queue;
};

using TelemetryPublisher = struct TelemetryPublisher {
  Topic topic;  // The going to have a list of the different ECU's that correlate to that topic
  string message_type;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
  rclcpp::TimerBase::SharedPtr timer;
};

class TelemetryNode : public rclcpp::Node {
public:
  explicit TelemetryNode(const vector<Topic>* topics);

  /**
    Creates the publishers and the TelemetryPublisher struct for each publisher. Uses contents
    returned by CanController::readFromLoc() to assign specific things to the TelemetryPublisher
    Then, assigns a standard callback
   **/

  TelemetryPublisher create_topic_publisher(Topic topic);

  void create_telemetry_subscriber();

  /**
   A more efficient way to create the Topic struct, that includes the topic it
   publishes to, and arbitration Id's that it listens to.

   Topic_One 123456 23456 (more_info_here)
   Topic_Two 345678 456789 (more_info_here)
 **/
  static vector<string> create_topics_from_text(string loc);

private:
  vector<TelemetryPublisher> publishers_;
  vector<Topic> topics_;
};

#endif //CANCONTROLLER_HPP