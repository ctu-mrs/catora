#include "mrs_formation_reshaping/robot.hpp"

using namespace mrs_formation_reshaping;

Robot::Robot(RobotState robot_state, int robot_id) {
  state_    = robot_state;
  robot_id_ = robot_id;
}

Robot::~Robot(void) {
}

void Robot::makeStep(double time_step) {

  if (set_trajectory_.empty()) {
    return;
  }

  time_since_trajectory_start_ += time_step;
  int n_steps = (int)round(time_since_trajectory_start_ / 0.2);
  ROS_DEBUG("[%s]: make step: n_steps = %d, set_trajectory_ size = %lu", ros::this_node::getName().c_str(), n_steps, set_trajectory_.size());

  if (n_steps >= set_trajectory_.size()) {
    n_steps = set_trajectory_.size() - 1;
  }

  state_.pos.x       = set_trajectory_[n_steps].x;
  state_.pos.y       = set_trajectory_[n_steps].y;
  state_.pos.z       = set_trajectory_[n_steps].z;
  state_.pos.heading = set_trajectory_[n_steps].heading;
}

void Robot::setTrajectory(const std::vector<Vector4d> &trajectory) {
  time_since_trajectory_start_ = 0.0;
  set_trajectory_              = trajectory;
}

void Robot::resetTrajectory() {
  time_since_trajectory_start_ = 0.0;
  makeStep(0.0);
}

std::vector<Vector4d> Robot::getTrajectory() {
  return set_trajectory_;
}

Vector4d Robot::getPose() {
  return state_.pos;
}

int Robot::getId() const {
  return robot_id_;
}

