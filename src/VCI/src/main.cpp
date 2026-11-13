int main(int argc, char* argv[]) {
  /*
    Topics are structs that will include more information about the specific topic
    for the telemetry node. initializing the vector array of topics and pass it to
    CanController
   */
  vector<Topic> topics;

  Topic steering;
  steering.name = "steering";
  steering.message_type = "N/A";

  topics.push_back(steering);

  pid_t serial_pid = fork();
  if (serial_pid == 0) {

  } else if (serial_pid > 0) {
    rclcpp::init(argc, argv);
    const auto telemetry_node = std::make_shared<TelemetryNode>(&topics);

    rclcpp::spin(telemetry_node);
    rclcpp::shutdown();
  }
}