#ifndef __TYPE_CONVERTOR_H__
#define __TYPE_CONVERTOR_H__

#include "mrs_msgs/TrajectoryReference.h"
#include "geometry_msgs/Point.h"
#include <Eigen/Dense>

namespace mrs_formation_reshaping
{

struct Vector3d
{
  double x;
  double y;
  double z;
};

struct Vector4d
{
  double x;
  double y;
  double z;
  double heading;

  double dist(Vector4d va) {
    return sqrt(pow(va.x - x, 2) + pow(va.y - y, 2) + pow(va.z - z, 2));
  }
};

struct Trajectory
{
  std::vector<geometry_msgs::Point> pos;
  std::vector<geometry_msgs::Point> vel;
  std::vector<geometry_msgs::Point> acc;
};

/**
 * @brief Class TypeConvertor
 */
class TypeConvertor {
public:
  /**
   * @brief constructor
   */
  TypeConvertor();

  /**
   * @brief destructor
   */
  ~TypeConvertor(void);

  static mrs_msgs::TrajectoryReference trajectoryToMrsMsgsTrajectory(const std::vector<Vector4d>& trajectory);

  static mrs_msgs::TrajectoryReference trajectoryToMrsMsgsTrajectory(const std::vector<Eigen::Vector3d>& trajectory);

  static mrs_msgs::Reference vector4dToReference(const Vector4d& point);

  static Eigen::Vector3d vector4dToEigen3d(const Vector4d& vector4d);

  static Vector4d eigen3dToVector4d(const Eigen::Vector3d& eigen_vec);

  static std::vector<mrs_msgs::Reference> vector4dToReference(const std::vector<Vector4d>& points);

  static geometry_msgs::Point vector4dToPoint(const Vector4d& v);

  static geometry_msgs::Point referenceToGeometryMsgsPoint(const mrs_msgs::Reference& point);

  static Vector4d referenceToVector4d(const mrs_msgs::Reference& point);

  static std::vector<Vector4d> referenceToVector4d(const std::vector<mrs_msgs::Reference>& points);

  static std::vector<geometry_msgs::Point> eigenVectors3dToGeometryMsgsPoints(const std::vector<Eigen::Vector3d>& vs);

  static std::vector<Eigen::Vector3d> geometryMsgsPointsToEigenVectors3d(const std::vector<geometry_msgs::Point>& ps);

  static geometry_msgs::Point eigenVectorToGeometryMsgsPoint(const Eigen::Vector3d& v);

  static geometry_msgs::Point vector4dToGeometryMsgsPoint(const Vector4d& ref);

  static Vector4d pointToVector4d(const geometry_msgs::Point& p);

  static double vector4dDist(const Vector4d& a, const Vector4d& b);

  static std::vector<Vector4d> trajectoryToVector4d(const Trajectory& trajectory);

  static std::vector<Vector4d> trajectoryE3dToVector4d(const std::vector<Eigen::Vector3d>& trajectory_3d);
};

}  // namespace mrs_formation_reshaping

#endif
