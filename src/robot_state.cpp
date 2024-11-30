#include "mrs_formation_reshaping/robot_state.hpp"

using namespace mrs_formation_reshaping;

RobotState::RobotState() {
}

RobotState::RobotState(const RobotState &robot_state) {
  pos = robot_state.pos;
  vel = robot_state.vel;
  acc = robot_state.acc;
}

RobotState::RobotState(const double x, const double y, const double z, const double heading) {
  pos.x       = x;
  pos.y       = y;
  pos.z       = z;
  pos.heading = heading;
}

RobotState::RobotState(const double x, const double y, const double z, const double heading, const double vx, const double vy, const double vz,
                       const double heading_rate, const double ax, const double ay, const double az, const double heading_acc) {
  pos.x       = x;
  pos.y       = y;
  pos.z       = z;
  pos.heading = heading;
  vel.x       = x;
  vel.y       = y;
  vel.z       = z;
  vel.heading = heading_rate;
  acc.x       = x;
  acc.y       = y;
  acc.z       = z;
  acc.heading = heading_acc;
}

RobotState::~RobotState(void) {
}

Eigen::Vector3d RobotState::getAbsPositionFromGoalAndRelativePose(const Vector4d &goal, const Vector4d &rel_pose, bool apply_heading) {

  Eigen::Vector3d abs_pose;

  if (apply_heading) {  // rotate whole formation with respect to heading of the leader
    abs_pose[0] = goal.x + cos(goal.heading) * rel_pose.x - sin(goal.heading) * rel_pose.y;
    abs_pose[1] = goal.y + sin(goal.heading) * rel_pose.x + cos(goal.heading) * rel_pose.y;
  } else {
    abs_pose[0] = goal.x + rel_pose.x;
    abs_pose[1] = goal.y + rel_pose.y;
  }

  abs_pose[2] = goal.z + rel_pose.z;

  return abs_pose;
}

Vector4d RobotState::getAbsPositionFromGoalAndRelativePoseVector4d(const Vector4d &goal, const Vector4d &rel_pose, bool apply_heading) {

  Vector4d abs_pose;

  if (apply_heading) {  // rotate whole formation with respect to heading of the leader
    abs_pose.x = goal.x + cos(goal.heading) * rel_pose.x - sin(goal.heading) * rel_pose.y;
    abs_pose.y = goal.y + sin(goal.heading) * rel_pose.x + cos(goal.heading) * rel_pose.y;
    abs_pose.z = goal.z + rel_pose.z;
  } else {
    abs_pose.x = goal.x + rel_pose.x;
    abs_pose.y = goal.y + rel_pose.y;
    abs_pose.z = goal.z + rel_pose.z;
  }

  abs_pose.heading = goal.heading + rel_pose.heading;

  return abs_pose;
}

