
#pragma once

#include <nav_msgs/msg/odometry.hpp>

namespace fusion::defines {
struct Observation {
  nav_msgs::msg::Odometry odom;
};
} // namespace fusion::defines