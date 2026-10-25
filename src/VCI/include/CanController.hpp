//
// Created by David Ola on 2026-10-24.
//


#ifndef CANCONTROLLER_HPP
#define CANCONTROLLER_HPP
using std::string;
using std::list;
using std::vector;

using PublisherPtr = rclcpp::Publisher<std_msgs::msg::String>::SharedPtr;

/**
  The SerialReceiver is going to receive a message, and then loop through a linkedlist
  of the TelemetryPublisher structs until it finds one that is meant for the message.

  Once the TelemetryPublisher receives the message, it publishes the message to the correct topic.

  So, TelemetryPublisher needs a way to
 **/
using TelemetryPublisher = struct TelemetryPublisher {
  std::string topic;  // The going to have a list of the different ECU's that correlate to that topic
  std::shared_ptr<PublisherPtr> publisher;
  std::shared_ptr<PublisherPtr>* next;
};

/**

 **/
using PublisherTable = struct PublisherTable {
  std::unique_ptr<TelemetryPublisher> head;
  int count;
};


class CanController : rclcpp::Node {

public:
  explicit CanController(string topics_list_loc);

  /**
    Creates the publishers and the TelemetryPublisher struct for each publisher. Uses contents
    returned by CanController::readFromLoc() to assign specific things to the TelemetryPublisher
    Then, assigns a standard callback
   **/
  void createPublishersByTopics(vector<string> topics_dict);

  /**
   Will read from a file containing the topic, and then the arbitration id's it publishes to

   Topic_One 123456 23456 (more_info_here)
   Topic_Two 345678 456789 (more_info_here)
 **/
  static vector<string> readFromLoc(string loc);

  void publishToTopic();

private:
  vector<string> topicsList_;
  std::shared_ptr<PublisherTable> publisherTable_;
  vector<PublisherPtr> publishers_;
};

#endif //CANCONTROLLER_HPP