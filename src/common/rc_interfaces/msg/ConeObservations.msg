# Batch capture time and original sensor frame. Each item also carries its headers.
std_msgs/Header header
string source_sensor
# Changes on process restart or observed ROS clock rewind.
string session_id
rc_interfaces/ConeObservation[] observations
