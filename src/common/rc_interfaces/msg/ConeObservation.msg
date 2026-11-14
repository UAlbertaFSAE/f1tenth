# Original measurement: capture time, sensor frame, metres and m^2.
std_msgs/Header source_header
string source_sensor
# Unique within a publisher session; NOT a persistent landmark ID.
string observation_id
geometry_msgs/Point source_position
bool source_position_valid
float64[9] source_covariance
bool covariance_known
# Capture-time coordinates in the requested target frame (normally odom).
std_msgs/Header header
geometry_msgs/Point position
bool position_valid
float64[9] covariance
# Empty when available; otherwise a machine-readable unavailability reason.
string unavailable_reason
# Probability of being a cone [0,1]; NaN means unknown, not zero.
float32 confidence
# Empty means unknown; otherwise blue/yellow/orange/large_orange.
string color
# Heuristic hypothesis, separate from measured color. Empty means no inference.
string inferred_color
# Empty when no inference; currently car_heading_left_right (base_link at capture).
string color_inference_method
# NaN until the heuristic has a calibrated probability; not cone confidence.
float32 inferred_color_confidence
# Camera image capture time/frame; source_header uses depth capture time when matched.
# Empty for lidar. Meaningful when has_box is true.
std_msgs/Header image_header
# Rectified image box [left, top, right, bottom] in pixels.
bool has_box
float64[4] box
# Snapshot of calibration used, including its frame and timestamp.
bool has_camera_info
sensor_msgs/CameraInfo camera_info
