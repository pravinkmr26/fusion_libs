#include <gtest/gtest.h>

#include "fusion_libs/adaptable_matrix.hpp"

#include <nav_msgs/msg/odometry.hpp>

TEST(AdaptableEMatrix, test_convariance_adapt) {
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = 10;

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
    0.01,       0.2,          0.0,        0.0,        0.0,      0.7,
    0.2,        0.0,          0.0,        0.0,        0.0,      0.20,
    0.0,        0.0,          0.0,        0.0,        0.0,      0.0,
    0.0,        0.0,          0.0,        0.0,        0.0,      0.0,
    0.0,        0.0,          0.0,        0.0,        0.0,      0. 0,
    0.7,        0.20,         0.0,        0.0,        0.0,      0.01
  };

  /**
   * expected output;
   * required_states = 0, 1, 5;
   * 0.01, 0.2, 0.7
   * 0.2, 0.01, 0.20
   * 0.7, 0.20, 0.01 
  */
  // clang-format on

  std::vector<MatrixIdx> required_states = {MatrixIdx::X, MatrixIdx::Y,
                                            MatrixIdx::C};
  AdaptableMatrix<double, 6, Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>
      matrix(odom.twist.covariance);
  auto output_matrix = matrix.getMatrix(required_states);

  EXPECT_EQ(output_matrix.rows(), required_states.size());
  EXPECT_EQ(output_matrix.cols(), required_states.size());
  EXPECT_EQ(output_matrix(0, 0), odom.twist.covariance[0]);
  EXPECT_EQ(output_matrix(0, 1), odom.twist.covariance[1]);
  EXPECT_EQ(output_matrix(0, 2), odom.twist.covariance[5]);
  EXPECT_EQ(output_matrix(1, 0), odom.twist.covariance[6]);
  EXPECT_EQ(output_matrix(1, 1), odom.twist.covariance[7]);
  EXPECT_EQ(output_matrix(1, 2), odom.twist.covariance[11]);
  EXPECT_EQ(output_matrix(2, 0), odom.twist.covariance[30]);
  EXPECT_EQ(output_matrix(2, 1), odom.twist.covariance[31]);
  EXPECT_EQ(output_matrix(2, 2), odom.twist.covariance[35]);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}