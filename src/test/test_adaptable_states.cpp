#include <gtest/gtest.h>

#include "fusion_libs/adaptable_state.hpp"


#include <nav_msgs/msg/odometry.hpp>

TEST(AdaptableStates, test_states_adapt) {
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = 10;
  odom.pose.pose.position.y = 5;
  odom.pose.pose.position.z = 2;

  odom.pose.pose.orientation = fusion::utils::quaternion::convert(
      fusion::utils::quaternion::getQuarternion(
          Eigen::Vector3d(3.0, 2.0, 1.0)));

  std::vector<StateIdx> required_states = {StateIdx::X, StateIdx::Y, StateIdx::Yaw};
  // AdaptableMatrix<double, 6, Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>
  //     matrix(odom.twist.covariance);
  FlexiState states(odom.pose.pose);
  auto output_vector = states.getMatrix(required_states);

  EXPECT_EQ(output_vector.size(), required_states.size());
  EXPECT_EQ(output_vector.x(), odom.pose.pose.position.x);
  EXPECT_EQ(output_vector.y(), odom.pose.pose.position.y);
  EXPECT_EQ(output_vector.z(), fusion::utils::quaternion::getEuler(odom.pose.pose.orientation).z());
}
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}