#include "state_vector.hpp"

#include "type_conversions.hpp"

namespace fusion::utils::cov {

using StateVector = fusion::utils::state::StateVector;
namespace quat_utils = fusion::utils::quaternion;

class OdomAdapter {
  nav_msgs::msg::Odometry input_;

public:
  OdomAdapter(nav_msgs::msg::Odometry &input) : input_(input) {}

  operator nav_msgs::msg::Odometry() {
    nav_msgs::msg::Odometry odom;
    odom.header = input_.header;
    return odom;
  }

  operator Eigen::VectorXd() {
    Eigen::VectorXd output(StateVector::Size);
    output[StateVector::X] = input_.pose.pose.position.x;
    output[StateVector::Y] = input_.pose.pose.position.y;
    output[StateVector::Yaw] =
        quat_utils::convert(input_.pose.pose.orientation).z();
    output[StateVector::Veloicy] = input_.twist.twist.linear.x;
    output[StateVector::YawRate] = input_.twist.twist.angular.z;
    return output;
  }
};
} // namespace fusion::utils::cov