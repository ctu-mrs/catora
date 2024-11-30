#ifndef __ROBOT_STATE_H__
#define __ROBOT_STATE_H__

#include <ros/ros.h>
#include "mrs_formation_reshaping/type_convertor.hpp"

namespace mrs_formation_reshaping
{

/**
 * @brief Class RobotState provides an object which collects all information about robot state
 */
class RobotState {
public:
  /**
   * @brief constructor which sets all initial coordinates to 0
   */
  RobotState(void);

  /**
   * @brief constructor
   */
  RobotState(const RobotState &robot_state);

  RobotState(const double x, const double y, const double z, const double heading, const double vx, const double vy, const double vz, const double heading_rate,
             const double ax, const double ay, const double az, const double heading_acc);

  RobotState(const double x, const double y, const double z, const double heading);

  /**
   * @brief destructor
   */
  ~RobotState(void);

  static Eigen::Vector3d getAbsPositionFromGoalAndRelativePose(const Vector4d &goal, const Vector4d &rel_pose, bool apply_heading);

  static Vector4d getAbsPositionFromGoalAndRelativePoseVector4d(const Vector4d &goal, const Vector4d &rel_pose, bool apply_heading);

  Vector4d pos;
  Vector4d vel;
  Vector4d acc;
};

}  // namespace mrs_formation_reshaping

#endif
