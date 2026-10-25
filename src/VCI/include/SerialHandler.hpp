#ifndef CAN_FRAME
#define CAN_FRAME

/*
  Creating a unique dataframe that is going to work for both my Publishers and Subscriber

  Header — marks the start of a message and identifies the protocol version.
  ID — says what kind of message this is and who it is from. - an alphabetical representation, not just nums
  Body — the actual payload.
  Checksum — so the receiver can tell if the message arrived intact.
*/

using CANFrame = struct CanFrame {};

/*
  This class will probably create a new child process that's going to constantly
  be reading from a specific port -> So this can be made super easy if its just socket programming,
  and then some other helper methods that allows it to read

  Reads the CANBUS message protocol (byte data) from a jetson nano port -> Likely will just use C++
  socket programming here
*/


/*
  The SerialPublisher can receive and then pipe information to the TelemetryPublisher.
  Then publish to

  The SerialReceiver can send information to the TelemetryPublisher using piping,
  or a mutex lock on
*/

class SerialReceiver {
public:
  SerialReceiver();

  /**
    SerialReceiver calls this method to publish to a specific topic, once it loops
    through all the topics that have the arbitration ID that it has received
   **/
  void publishToTopic(std::shared_ptr<PublisherPtr> publisher, string message);
private:
  vector<> arbitratio
};

/*
  Reads the CANFrame and transmits to CANBUS using serial port
*/
class SerialTransceiver {};

#endif
