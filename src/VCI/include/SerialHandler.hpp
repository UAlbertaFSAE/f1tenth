#ifndef CAN_FRAME
#define CAN_FRAME

/*
  Creating a unique dataframe that is going to work for both my Publishers and Subscriber

  Header — marks the start of a message and identifies the protocol version.
  ID — says what kind of message this is and who it is from. - an alphabetical representation, not just nums
  Body — the actual payload.
  Checksum — so the receiver can tell if the message arrived intact.
*/


/*
  This class will probably create a new child process that's going to constantly
  be reading from a specific port -> So this can be made super easy if its just socket programming,
  and then some other helper methods that allows it to read

  Reads the CANBUS message protocol (byte data) from a jetson nano port -> Likely will just use C++
  socket programming here
*/

class SerialReceiver {
public:
  SerialReceiver();

  /**
    SerialReceiver calls this method to the message queue of a specific topic. A function
    in the CanController checks the topic's message queue incrementally and publishes the message
    to the topic

    Waits for a message and publishes it incrementally

   **/
  void publishToTopic(std::string message, Topic topic);

private:
  int socket_;
  sockaddr_in address_;
};

/*
  Reads the CANFrame and transmits to CANBUS using serial port
*/
class SerialTransceiver {};

#endif
