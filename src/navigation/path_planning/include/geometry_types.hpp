// MIT License
//
// Copyright (c) 2026 UAlbertaFSAE
// SPDX-License-Identifier: MIT

#ifndef PATH_PLANNING_GEOMETRY_TYPES_HPP_
#define PATH_PLANNING_GEOMETRY_TYPES_HPP_

#include <array>
#include <cstddef>

#include "rclcpp/time.hpp"

namespace path_planning {
struct Point2D {
  double x = 0.0;
  double y = 0.0;
  bool is_left = false;  // true = left/blue boundary, false = right/yellow boundary
};

struct ConePair {
  Point2D start_pos;
  Point2D end_pos;
  Point2D mid_pos;
};

struct CarPose {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  bool has_odom = false;
  rclcpp::Time odom_stamp{0, 0, RCL_ROS_TIME};
};
}  // namespace path_planning

#endif  // PATH_PLANNING_GEOMETRY_TYPES_HPP_
