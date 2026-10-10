#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/*
  1. Create the data frame that the telemetry node is going to receive/transmit.
  So the telemetry node is also going to publish this data frame

    - CANBUS messages are going to be sent to a seperate process, and that process needs
    to communicate with the telemetry node. And the telemetry node needs to receive it and send
    information

    publishers have multiple topics
    subscribers only have one topic
    only one subscriber on a topic -> and my node is going to multiplex it to know
    where it should send the info to
  2.
*/

/*
  Different topics that the CanController node is going to publish to. This structure is created
  and added to a list inside of the CanController node. They're all created inside of a method

    - Has an array of arbitration ID's for that specific electrical control unit. This might need
    to be a class if I have to handle the arbitration ID's differently for each thing
*/
using UnitTopic = struct UnitTopic {};

/*
  Creating a unique dataframe that is going to work for both my Publishers and Subscriber

  Header — marks the start of a message and identifies the protocol version.
  ID — says what kind of message this is and who it is from.
  Body — the actual payload.
  Checksum — so the receiver can tell if the message arrived intact.
*/

using CANFrame = struct CanFrame {};

/*
  This class will probably create a new child process that's going to constantly
  be reading from a specific port -> So this can be made super easy if its just socket programming,
  and then some other helper methods that allows it to read
*/
class SerialReceiver {};

/*
  Inits multiple publishers. Sets the callback for every Publisher, to publisher to UnitTopic
  struct, using the CANFrame struct
*/
class TelemetryPublishers {};

class CanController : rclcpp::Node {
 public:
  CanController() : Node("CanController") {}

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main() {}
