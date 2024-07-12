#pragma once

#include <eigen3/Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>

namespace fusion::utils::quaternion {

static Eigen::Quaterniond convert(geometry_msgs::msg::Quaternion q) {
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

static Eigen::VectorXd getEuler(Eigen::Quaterniond q) {
  auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
  return euler;
}

static Eigen::Quaterniond getQuarternion(Eigen::Vector3d euler) {
  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ());
  return q;
}
} // namespace fusion::utils::quaternion