#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_with_covariance.hpp>

#include "type_conversions.hpp"

enum StateIdx { X = 0, Y = 1, Z = 2, Roll = 3, Pitch = 4, Yaw = 5 };

class FlexiState {
  Eigen::VectorXd states_;

public:
  FlexiState(const geometry_msgs::msg::Pose &pose) : states_(6) {
    auto euler = fusion::utils::quaternion::getEuler(pose.orientation);
    states_ << pose.position.x, pose.position.y, pose.position.z, euler.x(),
        euler.y(), euler.z();
  }

  FlexiState(const std::array<double, 6> array) {
    states_ = Eigen::Vector<double, 6>(array.data());
  }

  FlexiState(const Eigen::VectorXd &vector) { states_ = vector; }

  Eigen::VectorXd getMatrix(std::vector<StateIdx> &required_states){    
    Eigen::VectorXd output(required_states.size());
    for (size_t i = 0; i < required_states.size() ; i ++){
        output[i] = states_[required_states[i]];
    }
    return output;
  };
};