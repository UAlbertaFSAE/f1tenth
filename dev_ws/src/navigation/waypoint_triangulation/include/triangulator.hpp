#include "rc_interfaces/msg/cone.hpp"
#include "rc_interfaces/msg/cones.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class Triangulator : public rclcpp::Node {
 public:
  Triangulator();

 private:
  const rc_interfaces::msg::Cones::ConstSharedPtr last_cones;

  bool Triangulator::is_same_cone(const rc_interfaces::msg::Cone coneA,
                                  const rc_interfaces::msg::Cone coneB);
  rclcpp::Subscription<rc_interfaces::msg::Cones>::SharedPtr cone_subscriber;
  void read_cones(const rc_interfaces::msg::Cones::ConstSharedPtr cones);
  rc_interfaces::msg::Cone closest_cone(const rc_interfaces::msg::Cones::ConstSharedPtr cones);
  rc_interfaces::msg::Cone next_cone();
  bool has_new_cones(const rc_interfaces::msg::Cones::ConstSharedPtr cones);
};
