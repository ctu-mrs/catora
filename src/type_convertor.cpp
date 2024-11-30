#include "mrs_formation_reshaping/type_convertor.hpp"

using namespace mrs_formation_reshaping;

TypeConvertor::TypeConvertor() {
}

/* trajectoryToMrsMsgsTrajectory() //{ */
mrs_msgs::TrajectoryReference TypeConvertor::trajectoryToMrsMsgsTrajectory(const std::vector<Vector4d>& trajectory) {

  mrs_msgs::TrajectoryReference trajectory_ref;
  for (auto& point : trajectory) {
    mrs_msgs::Reference ref;
    ref.position.x = point.x;
    ref.position.y = point.y;
    ref.position.z = point.z;
    ref.heading    = point.heading;
    trajectory_ref.points.push_back(ref);
  }

  return trajectory_ref;
}
//}

/* trajectoryToMrsMsgsTrajectory() //{ */
mrs_msgs::TrajectoryReference TypeConvertor::trajectoryToMrsMsgsTrajectory(const std::vector<Eigen::Vector3d>& trajectory) {

  mrs_msgs::TrajectoryReference trajectory_ref;
  for (auto& point : trajectory) {
    mrs_msgs::Reference ref;
    ref.position.x = point[0];
    ref.position.y = point[1];
    ref.position.z = point[2];
    ref.heading    = 0.0;
    trajectory_ref.points.push_back(ref);
  }

  return trajectory_ref;
}
//}

/* vector4dToEigen3d() //{ */
Eigen::Vector3d TypeConvertor::vector4dToEigen3d(const Vector4d& vector4d) {

  return Eigen::Vector3d(vector4d.x, vector4d.y, vector4d.z);
}
//}

/* Eigen3dToVector4d() //{ */
Vector4d TypeConvertor::eigen3dToVector4d(const Eigen::Vector3d& eigen_vec) {

  Vector4d v;
  v.x       = eigen_vec[0];
  v.y       = eigen_vec[1];
  v.z       = eigen_vec[2];
  v.heading = 0.0;
  return v;
}
//}

/* vector4dToReference() //{ */
mrs_msgs::Reference TypeConvertor::vector4dToReference(const Vector4d& point) {

  mrs_msgs::Reference ref;
  ref.position.x = point.x;
  ref.position.y = point.y;
  ref.position.z = point.z;
  ref.heading    = point.heading;
  return ref;
}

std::vector<mrs_msgs::Reference> TypeConvertor::vector4dToReference(const std::vector<Vector4d>& points) {

  std::vector<mrs_msgs::Reference> refs;
  for (auto& p : points) {
    mrs_msgs::Reference ref;
    ref.position.x = p.x;
    ref.position.y = p.y;
    ref.position.z = p.z;
    ref.heading    = p.heading;
    refs.push_back(ref);
  }

  return refs;
}
//}

/* vector4dToPoint() //{ */
geometry_msgs::Point TypeConvertor::vector4dToPoint(const Vector4d& v) {

  geometry_msgs::Point point;
  point.x = v.x;
  point.y = v.y;
  point.z = v.z;

  return point;
}
//}

/* pointToVector4d() //{ */
Vector4d TypeConvertor::pointToVector4d(const geometry_msgs::Point& p) {

  Vector4d v;
  v.x       = p.x;
  v.y       = p.y;
  v.z       = p.z;
  v.heading = 0.0;

  return v;
}
//}

/* referenceToVector4d //{ */
Vector4d TypeConvertor::referenceToVector4d(const mrs_msgs::Reference& point) {

  Vector4d v;
  v.x       = point.position.x;
  v.y       = point.position.y;
  v.z       = point.position.z;
  v.heading = point.heading;
  return v;
}

std::vector<Vector4d> TypeConvertor::referenceToVector4d(const std::vector<mrs_msgs::Reference>& points) {

  std::vector<Vector4d> vs;
  for (auto& p : points) {
    Vector4d v;
    v.x       = p.position.x;
    v.y       = p.position.y;
    v.z       = p.position.z;
    v.heading = p.heading;
    vs.push_back(v);
  }

  return vs;
}
//}

/* eigenVectors3dToGeometryMsgsPoints() //{ */
std::vector<geometry_msgs::Point> TypeConvertor::eigenVectors3dToGeometryMsgsPoints(const std::vector<Eigen::Vector3d>& vs) {

  std::vector<geometry_msgs::Point> poses;
  for (auto& v : vs) {
    geometry_msgs::Point p;
    p.x = v[0];
    p.y = v[1];
    p.z = v[2];
    poses.push_back(p);
  }

  return poses;
}
//}

/* geometryMsgsPointsToEigenVectors3d() //{ */
std::vector<Eigen::Vector3d> TypeConvertor::geometryMsgsPointsToEigenVectors3d(const std::vector<geometry_msgs::Point>& ps) {

  std::vector<Eigen::Vector3d> poses;
  for (auto& p : ps) {
    Eigen::Vector3d v;
    v[0] = p.x;
    v[1] = p.y;
    v[2] = p.z;
    poses.push_back(v);
  }

  return poses;
}
//}

/* eigenVectorToGeometryMsgsPoint() //{ */
geometry_msgs::Point TypeConvertor::eigenVectorToGeometryMsgsPoint(const Eigen::Vector3d& v) {

  geometry_msgs::Point point;
  point.x = v.x();
  point.y = v.y();
  point.z = v.z();
  return point;
}
//}

/* referenceToGeometryMsgsPoint() //{ */
geometry_msgs::Point TypeConvertor::referenceToGeometryMsgsPoint(const mrs_msgs::Reference& ref) {

  geometry_msgs::Point point;
  point.x = ref.position.x;
  point.y = ref.position.y;
  point.z = ref.position.z;
  return point;
}
//}

/* vector4dToGeometryMsgsPoint() //{ */
geometry_msgs::Point TypeConvertor::vector4dToGeometryMsgsPoint(const Vector4d& ref) {

  geometry_msgs::Point point;
  point.x = ref.x;
  point.y = ref.y;
  point.z = ref.z;
  return point;
}
//}

/* vector4dDist //{ */
double TypeConvertor::vector4dDist(const Vector4d& a, const Vector4d& b) {
  return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}
//}

/* trajectoryToVector4d() //{ */
std::vector<Vector4d> TypeConvertor::trajectoryToVector4d(const Trajectory& trajectory) {

  std::vector<Vector4d> trajectory_leader;
  for (auto& point : trajectory.pos) {
    Vector4d pose;
    pose.x       = point.x;
    pose.y       = point.y;
    pose.z       = point.z;
    pose.heading = 0.0;
    trajectory_leader.push_back(pose);
  }

  return trajectory_leader;
}

/* trajectoryE3dToVector4d() //{ */
std::vector<Vector4d> TypeConvertor::trajectoryE3dToVector4d(const std::vector<Eigen::Vector3d>& trajectory_3d) {

  std::vector<Vector4d> trajectory_4d;
  for (auto& point_3d : trajectory_3d) {
    trajectory_4d.push_back(eigen3dToVector4d(point_3d));
  }

  return trajectory_4d;
}
//}

