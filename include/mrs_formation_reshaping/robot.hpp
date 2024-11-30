#ifndef __ROBOT_H__
#define __ROBOT_H__

#include "mrs_formation_reshaping/robot_state.hpp"

namespace mrs_formation_reshaping
{

/**
 * @brief Class Robot
 */
class Robot {
public:
  /**
   * @brief constructor
   */
  Robot(RobotState robot_state, int robot_id);

  /**
   * @brief destructor
   */
  ~Robot(void);

  int getId() const;

  void setTrajectory(const std::vector<Vector4d> &trajectory);

  void resetTrajectory();

  std::vector<Vector4d> getTrajectory();

  void makeStep(double time_step);

  Vector4d getPose();

  RobotState state_;

protected:
  int                   robot_id_;
  std::vector<Vector4d> set_trajectory_;
  double                time_since_trajectory_start_;
};

}  // namespace mrs_formation_reshaping

#endif
