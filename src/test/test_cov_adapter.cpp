#include <gtest/gtest.h>

#include "fusion_libs/cov_adapter.hpp"

using State = fusion::utils::cov::StateVector;
namespace quat_utils = fusion::utils::quaternion;

TEST(CovAdapter, test_init) {
  nav_msgs::msg::Odometry odom;
  Eigen::VectorXd value = fusion::utils::cov::OdomAdapter(odom);
  Eigen::VectorXd expect(fusion::utils::cov::StateVector::Size);
  expect.fill(0.0);
  EXPECT_EQ(value, expect);
}

TEST(CovAdapter, test_adapt_to_state_vector) {
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = 10;
  odom.pose.covariance.fill(0.0);
  odom.pose.covariance[0] = 0.01;
  odom.pose.covariance[7] = 0.1;
  Eigen::VectorXd state_vector = fusion::utils::cov::OdomAdapter(odom);
  EXPECT_EQ(state_vector[State::X], odom.pose.pose.position.x);
  EXPECT_EQ(state_vector[State::Y], odom.pose.pose.position.y);
  EXPECT_EQ(
      state_vector[State::Yaw],
      quat_utils::getEuler(quat_utils::convert(odom.pose.pose.orientation))
          .z());
  EXPECT_EQ(state_vector[State::Veloicy], odom.twist.twist.linear.x);
  EXPECT_EQ(state_vector[State::YawRate], odom.twist.twist.angular.z);
}

TEST(CovAdapter, test_convariance_adapt) {
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = 10;
  odom.pose.covariance.fill(0.0);
  odom.pose.covariance[0] = 0.01;
  odom.pose.covariance[7] = 0.1;

  // clang-format off
  odom.pose.covariance = {
    0.01,       0.0,          0.0,        0.0,        0.0,      0.0,
    0.0,        0.01,         0.0,        0.0,        0.0,      0.0,
    0.0,        0.0,          0.01,       0.0,        0.0,      0.0,
    0.0,        0.0,          0.0,        0.01,       0.0,      0.0,
    0.0,        0.0,          0.0,        0.0,        0.01,     0.0,
    0.0,        0.0,          0.0,        0.0,        0.0,      0.01
  };

  odom.twist.covariance = {
    0.01,       0.0,          0.0,        0.0,        0.0,      0.0,
    0.0,        0.0,          0.0,        0.0,        0.0,      0.0,
    0.0,        0.0,          0.0,        0.0,        0.0,      0.0,
    0.0,        0.0,          0.0,        0.0,        0.0,      0.0,
    0.0,        0.0,          0.0,        0.0,        0.0,      0.0,
    0.0,        0.0,          0.0,        0.0,        0.0,      0.01
  };
  // clang-format on
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}