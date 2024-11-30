/* includes and typedefs //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>

#include <mutex>
#include <tuple>
#include <limits>
#include <iostream>
#include <fstream>
#include <filesystem>

#include <mrs_lib/mutex.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/batch_visualizer.h>

#include <random>
#include <iterator>

#include "mrs_formation_reshaping/formation_reshaper.hpp"
#include "mrs_formation_reshaping/robot.hpp"

#include "mrs_formation_reshaping/TrajectoryMsg.h"
#include "mrs_formation_reshaping/SrvGetReshapingTrajectories.h"
#include "mrs_formation_reshaping/SrvGetAssignment.h"
#include "mrs_formation_reshaping/SrvReshapeFormation.h"

//}

namespace mrs_formation_reshaping
{

/* HELPER FUNCTIONS //{ */

/* random number generation //{ */
int getRandomInt(int start, int end) {
  std::random_device              rd;
  std::mt19937                    gen(rd());
  std::uniform_int_distribution<> distrib(start, end);
  return distrib(gen);
}
//}

/* getAbsoluteFilePath() //{ */
std::filesystem::path getAbsoluteFilePath(std::string folder, std::string filename) {
  std::filesystem::path folder_path        = folder;
  std::filesystem::path file_relative_path = filename;
  return folder_path / file_relative_path;
}
//}

/* getRandomPoint() //{ */
Eigen::Vector3d getRandomPoint(double dim_h, double dim_z, bool only_2d) {
  Eigen::Vector3d v;
  v[0] = -dim_h / 2.0 + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / dim_h));
  v[1] = -dim_h / 2.0 + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / dim_h));
  v[2] = only_2d ? 0.0 : -dim_z / 2.0 + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / dim_z));

  return v;
}
//}

/* getRandomVector() //{ */
std::vector<Eigen::Vector3d> getRandomVector(int size, double dim_h, double dim_z, double min_dist, bool only_2d = false) {
  std::vector<Eigen::Vector3d> vect;

  int counter = 0;
  while (vect.size() < size) {

    Eigen::Vector3d sp  = getRandomPoint(dim_h, dim_z, only_2d);
    bool            add = true;

    for (size_t spa = 0; spa < vect.size(); spa++) {
      if ((vect[spa] - sp).norm() < min_dist) {
        add = false;
        break;
      }
    }

    if (add) {
      vect.push_back(sp);
      counter = 0;
    }

    if (counter > 2000) {
      ROS_WARN_THROTTLE(1.0, "[FormationReshapingNodelet]: Clearing vector. Vector size = %lu", vect.size());
      vect.clear();
    }

    counter++;
  }

  return vect;
}
//}

/* loadCoordinatesFromTxt() //{ */
std::vector<Eigen::Vector3d> loadCoordinatesFromTxt(const std::string& file_path) {
  std::vector<Eigen::Vector3d> coordinates;

  // Open the TXT file
  std::ifstream file(file_path);
  if (!file.is_open()) {
    std::cerr << "Error: Could not open file " << file_path << std::endl;
    return coordinates;
  }

  std::string line;
  while (std::getline(file, line)) {
    std::stringstream line_stream(line);
    double            x, y, z;
    if (line_stream >> x >> y >> z) {
      Eigen::Vector3d coordinate(x, y, z);
      coordinates.push_back(coordinate);
    }
  }

  // Close the file
  file.close();

  ROS_INFO("[FormationReshapingNodelet]: %lu coordinates loaded from %s", coordinates.size(), file_path.c_str());
  return coordinates;
}
//}

//}

/* class FormationReshapingNodelet //{ */
class FormationReshapingNodelet : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle _nh;

  // Variables
  FormationReshaper formation_reshaper_;

  bool is_initialized_ = false;
  bool is_busy_ = false;

  bool run_catora_vs_lsap_test_;
  bool generate_trajectories_for_loaded_configurations_;

  double time_step_;
  double maximum_velocity_;
  double maximum_acceleration_;

  // ROS handlers
  ros::ServiceServer srv_run_visualization_;
  ros::ServiceServer srv_reset_trajectories_;

  ros::ServiceServer srv_reshape_formation_;
  ros::ServiceServer srv_get_assignment_;
  ros::ServiceServer srv_get_reshaping_trajectories_;

  // ROS Timers
  ros::Timer main_timer_;
  ros::Timer vis_publisher_timer_;

  // Main timer functions
  void runLsapVsCatoraTest(const ros::TimerEvent& event);

  void generateTrajectoriesForConfigurationsFromFile(const ros::TimerEvent& event);

  void runFormationReshapingServer(const ros::TimerEvent& event);

  // Service callbacks
  bool callbackReshapeFormation(mrs_formation_reshaping::SrvReshapeFormation::Request& req, mrs_formation_reshaping::SrvReshapeFormation::Response& res);

  bool callbackGetAssignment(mrs_formation_reshaping::SrvGetAssignment::Request& req, mrs_formation_reshaping::SrvGetAssignment::Response& res);

  bool callbackGetReshapingTrajectories(mrs_formation_reshaping::SrvGetReshapingTrajectories::Request&  req,
                                        mrs_formation_reshaping::SrvGetReshapingTrajectories::Response& res);

  bool callbackRunVisualization(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  bool callbackResetTrajectories(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  // Trajectory generation and manipulation
  void addStopsToTrajectories(std::vector<std::vector<Eigen::Vector3d>>& trajectories, double stop_duration, double time_step);

  double getTrajectoriesMinimumDistance(const std::vector<Eigen::Vector3d>& starts, const std::vector<Eigen::Vector3d>& goals,
                                        const std::vector<std::pair<int, int>>& matching);

  void getAssignmentCharacteristics(const std::vector<Eigen::Vector3d>& starts, const std::vector<Eigen::Vector3d>& goals,
                                    const std::vector<std::pair<int, int>>& matching, double& max_trajectory_length, double& sum_trajectory_lengths_squared,
                                    double& min_mutual_dist);

  void saveTrajectoryToFile(std::string filename, const std::vector<Eigen::Vector3d>& trajectory);

  void saveSetOfTrajectoriesToFile(std::string filename, const std::vector<std::vector<Eigen::Vector3d>>& trajectories);

  void publishVisualizations(const ros::TimerEvent& event);

  std::vector<Eigen::Vector3d> getRobotPositions();

  // Visualization
  bool visualize_assignment_;
  bool trajectories_set_  = false;
  bool run_visualization_ = false;

  double visualization_real_time_factor_;
  double visualization_timer_rate_ = 10;

  std::vector<Eigen::Vector3d>     start_points_;
  std::vector<Eigen::Vector3d>     end_points_;
  std::vector<std::pair<int, int>> assignment_catora_;
  std::vector<std::pair<int, int>> assignment_lsap_;

  std::mutex mutex_comparison_visualizations_;

  std::shared_ptr<std::vector<Robot>> robots_;
  std::mutex                          mutex_robots_;

  void clearBvBuffers();
  void clearAssignmentVisualizations();
  void publishRobots();
  void publishTrajectories();
  void publishLsap();
  void publishCatora();

  void updateRobots(const std::vector<std::vector<Eigen::Vector3d>>& trajectories);
  void updateRobots(int goal_number_of_robots);

  mrs_lib::BatchVisualizer bv_robots_;
  std::mutex               mutex_bv_robots_;

  mrs_lib::BatchVisualizer bv_ref_trajectories_;
  std::mutex               mutex_bv_ref_trajectories_;

  mrs_lib::BatchVisualizer bv_lsap_;
  std::mutex               mutex_bv_lsap_;

  mrs_lib::BatchVisualizer bv_catora_;
  std::mutex               mutex_bv_catora_;

  std::string global_frame_;

  // Reshaping test
  double test_area_horizontal_dimension_;
  double test_area_vertical_dimension_;

  int n_robots_min_;
  int n_robots_max_;

  // IO variables
  std::string              formation_folder_;
  std::vector<std::string> formation_files_;

  bool        save_trajectories_to_file_;
  std::string trajectories_output_file_;
  std::string trajectories_output_folder_;
};
//}

/* onInit() //{ */
void FormationReshapingNodelet::onInit() {

  _nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::Time::waitForValid();

  // Get parameters from config file
  mrs_lib::ParamLoader param_loader(_nh, "FormationReshapingNodelet");

  param_loader.loadParam("global_frame", global_frame_);
  param_loader.loadParam("constraints/max_vel", maximum_velocity_);
  param_loader.loadParam("constraints/max_acc", maximum_acceleration_);
  param_loader.loadParam("time_step", time_step_);
  param_loader.loadParam("trajectories_out_file", trajectories_output_file_);
  param_loader.loadParam("trajectories_out_folder", trajectories_output_folder_);
  param_loader.loadParam("save_trajectories_to_file", save_trajectories_to_file_);
  param_loader.loadParam("visualization/real_time_factor", visualization_real_time_factor_);
  param_loader.loadParam("testing/number_of_robots/min", n_robots_min_);
  param_loader.loadParam("testing/number_of_robots/max", n_robots_max_);
  param_loader.loadParam("testing/area/horizontal_dimension", test_area_horizontal_dimension_);
  param_loader.loadParam("testing/area/vertical_dimension", test_area_vertical_dimension_);
  param_loader.loadParam("formation_folder", formation_folder_);
  param_loader.loadParam("formation_files", formation_files_);
  param_loader.loadParam("run_catora_vs_lsap_test", run_catora_vs_lsap_test_);
  param_loader.loadParam("generate_trajectories_for_loaded_configurations", generate_trajectories_for_loaded_configurations_);

  if (!param_loader.loadedSuccessfully()) {
    NODELET_ERROR("[FormationReshapingNodelet]: Some compulsory parameters were not loaded successfully, ending the node.");
    ros::shutdown();
  }

  if (run_catora_vs_lsap_test_ && generate_trajectories_for_loaded_configurations_) {
    ROS_WARN("[FormationReshapingNodelet]: CAT-ORA vs LSAP test and trajectory generation for loaded configurations cannot be run at the same time.");
    ROS_WARN(
        "[FormationReshapingNodelet]: CAT-ORA vs LSAP test will be prioritized. For generating trajectories for loaded configurations, set parameter "
        "run_catora_vs_lsap_test to false.");
  }

  srv_run_visualization_  = _nh.advertiseService("run_visualization", &FormationReshapingNodelet::callbackRunVisualization, this);
  srv_reset_trajectories_ = _nh.advertiseService("reset_trajectories", &FormationReshapingNodelet::callbackResetTrajectories, this);

  srv_reshape_formation_          = _nh.advertiseService("reshape_formation", &FormationReshapingNodelet::callbackReshapeFormation, this);
  srv_get_reshaping_trajectories_ = _nh.advertiseService("get_reshaping_trajectories", &FormationReshapingNodelet::callbackGetReshapingTrajectories, this);
  srv_get_assignment_             = _nh.advertiseService("get_reshaping_assignment", &FormationReshapingNodelet::callbackGetAssignment, this);

  // | -------------------- batch visualizers ------------------- |

  bv_robots_ = mrs_lib::BatchVisualizer(_nh, "visualize_robots", global_frame_);
  bv_robots_.setPointsScale(1.2);
  bv_robots_.setLinesScale(0.1);

  bv_lsap_ = mrs_lib::BatchVisualizer(_nh, "visualize_lsap", global_frame_);
  bv_lsap_.setPointsScale(0.5);
  bv_lsap_.setLinesScale(0.1);

  bv_catora_ = mrs_lib::BatchVisualizer(_nh, "visualize_catora", global_frame_);
  bv_catora_.setPointsScale(0.5);
  bv_catora_.setLinesScale(0.1);

  bv_ref_trajectories_ = mrs_lib::BatchVisualizer(_nh, "visualize_ref_trajectories", global_frame_);
  bv_ref_trajectories_.setPointsScale(0.3);
  bv_ref_trajectories_.setLinesScale(0.1);

  formation_reshaper_ = FormationReshaper(maximum_acceleration_, maximum_velocity_, time_step_);

  if (run_catora_vs_lsap_test_) {
    main_timer_           = _nh.createTimer(ros::Rate(1000), &FormationReshapingNodelet::runLsapVsCatoraTest, this);
    visualize_assignment_ = true;
  } else if (generate_trajectories_for_loaded_configurations_) {
    main_timer_           = _nh.createTimer(ros::Rate(2), &FormationReshapingNodelet::generateTrajectoriesForConfigurationsFromFile, this);
    visualize_assignment_ = false;
  } else {
    main_timer_ = _nh.createTimer(ros::Rate(2), &FormationReshapingNodelet::runFormationReshapingServer, this);
  }

  vis_publisher_timer_ = _nh.createTimer(ros::Rate(visualization_timer_rate_), &FormationReshapingNodelet::publishVisualizations, this);

  NODELET_INFO_ONCE("[FormationReshapingNodelet] Nodelet initialized");

  NODELET_INFO_ONCE("[FormationReshapingNodelet] Initializing robots.");
  robots_ = std::make_shared<std::vector<Robot>>();
  for (int k = 0; k < 100; k++) {  // initialize some robots, will be overwritten by incoming requests
    RobotState state(k / 5.0, -15.0 + k % 5 * 3.0, 2.0, 0.0);
    Robot      rob(state, k);
    robots_->push_back(rob);
  }

  is_initialized_ = true;
  NODELET_INFO("[FormationReshapingNodelet] Node initialized.");
}
//}

/* MAIN TIMER CALLBACKS //{ */

/* runFormationReshapingServer() //{*/
void FormationReshapingNodelet::runFormationReshapingServer(const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  if (!is_busy_) { 
    ROS_INFO_THROTTLE(5.0, "[FormationReshapingNodelet]: Waiting for the next request.");
  } else { 
    ROS_INFO_THROTTLE(5.0, "[FormationReshapingNodelet]: Processing request.");
  }
}
/*//}*/

/* runLsapVsCatoraTest() //{ */
void FormationReshapingNodelet::runLsapVsCatoraTest(const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  int    n_robots_min = std::max(1, n_robots_min_);
  int    n_robots_max = std::max(n_robots_min_, n_robots_max_);
  int    n_points     = getRandomInt(n_robots_min, n_robots_max);
  double dim_h        = test_area_horizontal_dimension_;
  double dim_z        = test_area_vertical_dimension_;

  double                       dist_thr = 1.0;
  std::vector<Eigen::Vector3d> start_points;
  std::vector<Eigen::Vector3d> end_points;

  ROS_INFO("[FormationReshapingNodelet]: -------------------------------------------------------------------------------");
  ROS_INFO("[FormationReshapingNodelet]: Running LSAP vs CATORA test: n_points = %d, dist_thr = %.2f", n_points, dist_thr);
  start_points = getRandomVector(n_points, dim_h, dim_z, dist_thr, false);
  end_points   = getRandomVector(n_points, dim_h, dim_z, dist_thr, false);

  auto   result_lsap        = formation_reshaper_.getLsapAssignment(start_points, end_points);
  auto   assignment_lsap    = result_lsap.first;
  double solution_time_lsap = result_lsap.second[0];

  auto   result_catora        = formation_reshaper_.getCatoraAssignment(start_points, end_points);
  auto   assignment_catora    = result_catora.first;
  double solution_time_catora = result_catora.second[0];

  double max_trajectory_length_lsap, sum_trajectory_lengths_squared_lsap, min_mutual_dist_lsap;
  double max_trajectory_length_catora, sum_trajectory_lengths_squared_catora, min_mutual_dist_catora;

  getAssignmentCharacteristics(start_points, end_points, assignment_lsap, max_trajectory_length_lsap, sum_trajectory_lengths_squared_lsap,
                               min_mutual_dist_lsap);
  getAssignmentCharacteristics(start_points, end_points, assignment_catora, max_trajectory_length_catora, sum_trajectory_lengths_squared_catora,
                               min_mutual_dist_catora);

  double solution_length_ratio = max_trajectory_length_lsap / max_trajectory_length_catora;

  {
    std::scoped_lock lock(mutex_comparison_visualizations_);
    start_points_      = start_points;
    end_points_        = end_points;
    assignment_catora_ = assignment_catora;
    assignment_lsap_   = assignment_lsap;
  }

  ROS_INFO("[FormationReshapingNodelet]: CAT-ORA vs LSAP: Solution length ratio = %.2f, min. dist ratio = %.2f, computational time ratio = %.2f.",
           max_trajectory_length_catora / max_trajectory_length_lsap, min_mutual_dist_catora / min_mutual_dist_lsap, solution_time_catora / solution_time_lsap);

  ROS_INFO("[FormationReshapingNodelet]: Waiting for key press.");
  getchar();
}
//}

/* generateTrajectoriesForConfigurationsFromFile() //{ */
void FormationReshapingNodelet::generateTrajectoriesForConfigurationsFromFile(const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  if (trajectories_set_) {

    ROS_INFO("[FormationReshapingNodelet]: Press key to replay the visualization.");
    getchar();
    std::scoped_lock lock(mutex_robots_);
    for (auto& robot : *robots_) {
      robot.resetTrajectory();
    }

    return;
  }

  std::vector<std::vector<Eigen::Vector3d>> configurations;

  ROS_INFO("[FormationReshapingNodelet]: Loading coordinates.");

  // load csv files with formations
  for (auto& file : formation_files_) {

    std::vector<Eigen::Vector3d> configuration = loadCoordinatesFromTxt(getAbsoluteFilePath(formation_folder_, file));

    if (configuration.empty()) {
      ROS_ERROR("[FormationReshapingNodelet]: Empty configuration loaded. Cannot generate reshaping trajectories.");
      return;
    }

    if (!configurations.empty() && configuration.size() != configurations[0].size()) {
      ROS_ERROR("[FormationReshapingNodelet]: The number of elements in individual configurations does not match. Cannot generate reshaping trajectories.");
      return;
    }

    configurations.push_back(configuration);
  }

  ROS_INFO("[FormationReshapingNodelet]: Coordinates loaded.");

  std::vector<std::vector<Eigen::Vector3d>> trajectories_whole(configurations[0].size());

  for (size_t c = 1; c < configurations.size(); c++) {
    ROS_INFO("[FormationReshapingNodelet]: Starting computation for configuration %lu", c);
    std::vector<std::vector<Eigen::Vector3d>> trajectories = formation_reshaper_.getReshapingTrajectoriesCatora(configurations[c - 1], configurations[c]);

    for (size_t t = 0; t < trajectories.size(); t++) {
      trajectories_whole[t].insert(trajectories_whole[t].end(), trajectories[t].begin(), trajectories[t].end());
    }
  }

  ROS_INFO("[FormationReshapingNodelet]: Trajectories generated. Makespan = %.2f", time_step_ * trajectories_whole[0].size());

  if (save_trajectories_to_file_) {
    saveSetOfTrajectoriesToFile(getAbsoluteFilePath(trajectories_output_folder_, trajectories_output_file_), trajectories_whole);
  }

  // create robots
  {
    std::scoped_lock lock(mutex_robots_);
    updateRobots(trajectories_whole);

    for (size_t r = 0; r < trajectories_whole.size(); r++) {
      robots_->at(r).setTrajectory(TypeConvertor::trajectoryE3dToVector4d(trajectories_whole[r]));
    }
  }

  ROS_INFO("[FormationReshapingNodelet]: Trajectories set.");
  run_visualization_ = true;
  trajectories_set_  = true;
}
//}

//}

/* SERVICE CALLBACKS //{ */

/* callbackGetAssignment() //{ */
bool FormationReshapingNodelet::callbackGetAssignment(mrs_formation_reshaping::SrvGetAssignment::Request&  req,
                                                      mrs_formation_reshaping::SrvGetAssignment::Response& res) {

  ROS_INFO("[FormationReshapingNodelet]: Get assignment service called.");

  res.success = false;

  if (!is_initialized_) {
    ROS_WARN("[FormationReshapingNodelet]: Service cannot be called. Not initialized.");
    res.message = "uninitialized, cannot process request";
    return true;
  }

  if (req.initial_configurations.size() != req.goal_configurations.size()) {
    ROS_WARN("[FormationReshapingNodelet]: Size of initial and goal configurations does not match. Cannot process request.");
    res.message = "size of initial and goal configuration does not match";
    return true;
  }

  is_busy_ = true;

  std::vector<Eigen::Vector3d> initial_configuration = TypeConvertor::geometryMsgsPointsToEigenVectors3d(req.initial_configurations);
  std::vector<Eigen::Vector3d> final_configuration   = TypeConvertor::geometryMsgsPointsToEigenVectors3d(req.goal_configurations);

  auto                             result     = formation_reshaper_.getCatoraAssignment(initial_configuration, final_configuration);
  std::vector<std::pair<int, int>> assignment = result.first;

  res.mapping.resize(assignment.size());
  for (auto& m : assignment) {
    res.mapping[m.first] = m.second;
  }

  if (req.publish_visualization) {
    std::scoped_lock lock(mutex_comparison_visualizations_);
    visualize_assignment_ = true;
    start_points_         = initial_configuration;
    end_points_           = final_configuration;
    assignment_catora_    = assignment;
    assignment_lsap_.clear();
    clearBvBuffers();
  }

  res.success = true;
  res.message = "success";
  is_busy_ = false;
  return true;
}
//}

/* callbackGetReshapingTrajectories() //{ */
bool FormationReshapingNodelet::callbackGetReshapingTrajectories(mrs_formation_reshaping::SrvGetReshapingTrajectories::Request&  req,
                                                                 mrs_formation_reshaping::SrvGetReshapingTrajectories::Response& res) {

  ROS_INFO("[FormationReshapingNodelet]: Get reshaping trajectories service called.");

  res.success = false;

  if (!is_initialized_) {
    ROS_WARN("[FormationReshapingNodelet]: Service cannot be called. Not initialized.");
    res.message = "uninitialized, cannot process request";
    return true;
  }

  if (req.initial_configurations.size() != req.goal_configurations.size()) {
    ROS_WARN("[FormationReshapingNodelet]: Size of initial and goal configurations does not match. Cannot process request.");
    res.message = "size of initial and goal configuration does not match";
    return true;
  }

  if (req.max_acceleration < 1e-3 || req.max_velocity < 1e-3 || req.trajectory_dt < 1e-3) {
    ROS_WARN("[FormationReshapingNodelet]: The allowed maximum acceleration, maximum velocity or traejctory dt is too small. Cannot process request.");
    res.message = "the provided constraints are not valid";
    return true;
  }

  is_busy_ = true;

  std::vector<Eigen::Vector3d> initial_configuration = TypeConvertor::geometryMsgsPointsToEigenVectors3d(req.initial_configurations);
  std::vector<Eigen::Vector3d> final_configuration   = TypeConvertor::geometryMsgsPointsToEigenVectors3d(req.goal_configurations);

  ROS_INFO("[FormationReshapingNodelet]: Obtained request with %lu initial and %lu final configurations", initial_configuration.size(),
           final_configuration.size());

  formation_reshaper_.setConstraintsAndDt(req.max_velocity, req.max_acceleration, req.trajectory_dt);
  auto trajectories = formation_reshaper_.getReshapingTrajectoriesCatora(initial_configuration, final_configuration);

  res.trajectories.resize(trajectories.size());
  for (size_t k = 0; k < res.trajectories.size(); k++) {
    res.trajectories[k].waypoints = TypeConvertor::eigenVectors3dToGeometryMsgsPoints(trajectories.at(k));
  }

  if (req.publish_visualization) {
    std::scoped_lock lock(mutex_robots_);
    visualize_assignment_ = false;
    clearAssignmentVisualizations();

    updateRobots(trajectories);

    for (size_t r = 0; r < trajectories.size(); r++) {
      robots_->at(r).setTrajectory(TypeConvertor::trajectoryE3dToVector4d(trajectories.at(r)));
    }
  }

  trajectories_set_  = true;
  run_visualization_ = true;

  res.success = true;
  res.message = "success";
  is_busy_ = false;
  return true;
}
//}

/* callbackReshapeFormation() //{ */
bool FormationReshapingNodelet::callbackReshapeFormation(mrs_formation_reshaping::SrvReshapeFormation::Request&  req,
                                                         mrs_formation_reshaping::SrvReshapeFormation::Response& res) {
  ROS_INFO("[FormationReshapingNodelet]: Reshape Formation service called.");

  res.success = false;

  if (!is_initialized_) {
    ROS_WARN("[FormationReshapingNodelet]: Service cannot be called. Not initialized.");
    res.message = "uninitialized, cannot process request";
    return true;
  }

  is_busy_ = true;

  std::scoped_lock lock(mutex_robots_);

  std::vector<Eigen::Vector3d> final_configuration = TypeConvertor::geometryMsgsPointsToEigenVectors3d(req.goal_configurations);

  if (robots_->size() != final_configuration.size()) {
    // add or remove random points from initial configuration to match number of robots in final configuration
    updateRobots(final_configuration.size());
  }

  std::vector<Eigen::Vector3d> initial_configuration = getRobotPositions();

  std::vector<std::vector<Eigen::Vector3d>> trajectories = formation_reshaper_.getReshapingTrajectoriesCatora(initial_configuration, final_configuration);

  for (size_t r = 0; r < trajectories.size(); r++) {
    robots_->at(r).setTrajectory(TypeConvertor::trajectoryE3dToVector4d(trajectories[r]));
  }

  ROS_INFO("[FormationReshapingNodelet]: Trajectories set.");
  trajectories_set_     = true;
  visualize_assignment_ = false;
  clearAssignmentVisualizations();

  run_visualization_ = true;

  res.success = true;
  res.message = "success";

  is_busy_ = false;
  return true;
}
//}

/* callbackRunVisualization() //{ */
bool FormationReshapingNodelet::callbackRunVisualization(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
  ROS_INFO("[FormationReshapingNodelet]: Run visualization service called.");
  res.success = false;

  if (!is_initialized_) {
    ROS_WARN("[FormationReshapingNodelet]: Service cannot be called. Not initialized.");
    res.message = "uninitialized, cannot process request";
    return true;
  }

  if (req.data) {
    if (run_visualization_) {
      ROS_WARN("[FormationReshapingNodelet]: Service cannot be called. Visualization is already running.");
      res.message = "visualization is already running";
      return true;
    } else {
      run_visualization_ = true;
      ROS_WARN("[FormationReshapingNodelet]: Visualization started.");
    }
  } else {
    if (!run_visualization_) {
      ROS_WARN("[FormationReshapingNodelet]: Service cannot be called. Visualization is already stopped.");
      res.message = "visualization is already running";
      return true;
    } else {
      run_visualization_ = false;
      ROS_WARN("[FormationReshapingNodelet]: Visualization stopped");
    }
  }

  res.success = true;
  res.message = "success";
  return true;
}
//}

/* callbackResetTrajectories() //{ */
bool FormationReshapingNodelet::callbackResetTrajectories(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  ROS_INFO("[FormationReshapingNodelet]: Reset trajectories service called.");
  res.success = false;

  if (!is_initialized_) {
    ROS_WARN("[FormationReshapingNodelet]: Service cannot be called. Not initialized.");
    res.message = "uninitialized, cannot process request";
    return res.success;
  }

  std::scoped_lock lock(mutex_robots_);
  run_visualization_ = false;
  for (auto& robot : *robots_) {
    robot.resetTrajectory();
  }

  res.success = true;
  res.message = "success";
  return true;
}
//}

//}

/* TRAJECTORY MANIPULATION - evaluation, ouput generation //{ */

/* saveTrajectoryToFile() //{ */
void FormationReshapingNodelet::saveTrajectoryToFile(std::string filename, const std::vector<Eigen::Vector3d>& trajectory) {
  std::ofstream output;
  output.open(filename);

  if (output.is_open()) {
    std::stringstream line;
    for (size_t k = 0; k < trajectory.size(); k++) {
      line.str(std::string());
      line << trajectory[k][0] << " " << trajectory[k][1] << " " << trajectory[k][2] << " " << 0.0 << "\n";
      output << line.str();
    }
    output.close();
  } else {
    ROS_ERROR("[FormationReshapingNodelet]: Unable to open file %s.", filename.c_str());
  }
}
//}

/* saveSetOfTrajectoriesToFile() //{ */
void FormationReshapingNodelet::saveSetOfTrajectoriesToFile(std::string filename, const std::vector<std::vector<Eigen::Vector3d>>& trajectories) {
  std::string sub_string       = filename.substr(0, filename.find(".", 0));
  std::string current_filename = sub_string + "_xxx.txt";
  ROS_INFO("[FormationReshapingNodelet]: Saving trajectories to files %s", current_filename.c_str());
  for (size_t k = 0; k < trajectories.size(); k++) {
    std::ostringstream ostr;
    ostr << std::setw(3) << std::setfill('0') << k;
    current_filename = sub_string + "_" + ostr.str() + ".txt";
    saveTrajectoryToFile(current_filename, trajectories[k]);
  }

  ROS_INFO("[FormationReshapingNodelet]: Plans saved.");
}
//}

/* addStopsToTrajectories() //{ */
void FormationReshapingNodelet::addStopsToTrajectories(std::vector<std::vector<Eigen::Vector3d>>& trajectories, double stop_duration, double time_step) {
  int n_steps = ceil(stop_duration / time_step);
  for (auto& t : trajectories) {
    for (int k = 0; k < n_steps; k++) {
      t.push_back(t.back());
    }
  }
}
//}

/* getAssignmentCharacteristics() //{ */
void FormationReshapingNodelet::getAssignmentCharacteristics(const std::vector<Eigen::Vector3d>& starts, const std::vector<Eigen::Vector3d>& goals,
                                                             const std::vector<std::pair<int, int>>& matching, double& max_trajectory_length,
                                                             double& sum_trajectory_lengths_squared, double& min_mutual_dist) {

  max_trajectory_length          = 0;
  sum_trajectory_lengths_squared = 0;
  min_mutual_dist                = DBL_MAX;

  for (auto& m : matching) {
    double dist = (starts[m.first] - goals[m.second]).norm();
    if (dist > max_trajectory_length) {
      max_trajectory_length = dist;
    }

    sum_trajectory_lengths_squared += pow(dist, 2);
  }

  min_mutual_dist = getTrajectoriesMinimumDistance(starts, goals, matching);
}
//}

/* getTrajectoriesMinimumDistance() //{ */
double FormationReshapingNodelet::getTrajectoriesMinimumDistance(const std::vector<Eigen::Vector3d>& starts, const std::vector<Eigen::Vector3d>& goals,
                                                                 const std::vector<std::pair<int, int>>& matching) {

  // get maximum path length and z difference
  double                                    dist_min = 0.0;
  std::vector<std::vector<Eigen::Vector3d>> trajectories;
  trajectories.resize(starts.size());
  int n_points = 100;

  for (size_t m = 0; m < matching.size(); m++) {

    double dist = (starts[matching[m].first] - goals[matching[m].second]).norm();

    for (int k = 0; k <= n_points; k++) {
      double th = k / double(n_points);
      trajectories[m].push_back((1 - th) * starts[matching[m].first] + th * goals[matching[m].second]);
    }
  }

  double          min_dist = DBL_MAX;
  double          dist;
  int             pa_min, pb_min;
  Eigen::Vector3d point_min_pa, point_min_pb;

  for (size_t pa = 0; pa < trajectories.size(); pa++) {
    for (size_t pb = pa + 1; pb < trajectories.size(); pb++) {

      for (size_t k = 0; k < trajectories[pa].size(); k++) {
        dist = (trajectories[pa][k] - trajectories[pb][k]).norm();
        if (dist < min_dist) {
          min_dist     = dist;
          pa_min       = pa;
          pb_min       = pb;
          point_min_pa = trajectories[pa][k];
          point_min_pb = trajectories[pb][k];
        }
      }
    }
  }

  ROS_DEBUG_COND(starts.size() > 1,
                 "[FormationReshapingNodelet]: Result min_dist = %.2f for trajectories %d and %d, points [%.2f, %.2f, %.2f] and [%.2f, %.2f, %.2f]", min_dist,
                 pa_min, pb_min, point_min_pa[0], point_min_pa[1], point_min_pa[2], point_min_pb[0], point_min_pb[1], point_min_pb[2]);
  ROS_DEBUG_COND(
      starts.size() > 1,
      "[FormationReshapingNodelet]: Min dist start_a [%d] = [%.2f, %.2f, %.2f], goal_a [%d] = [%.2f, %.2f, %.2f], start_b [%d] = [%.2f, %.2f, %.2f], goal_b "
      "[%d] "
      "= [%.2f, %.2f, %.2f]",
      matching[pa_min].first, starts[matching[pa_min].first][0], starts[matching[pa_min].first][1], starts[matching[pa_min].first][2], matching[pa_min].second,
      goals[matching[pa_min].second][0], goals[matching[pa_min].second][1], goals[matching[pa_min].second][2], matching[pb_min].first,
      starts[matching[pb_min].first][0], starts[matching[pb_min].first][1], starts[matching[pb_min].first][2], matching[pb_min].second,
      goals[matching[pb_min].second][0], goals[matching[pb_min].second][1], goals[matching[pb_min].second][2]);

  return min_dist;
}
//}

/* getRobotPositions() //{ */
std::vector<Eigen::Vector3d> FormationReshapingNodelet::getRobotPositions() {

  std::vector<Eigen::Vector3d> positions;

  for (auto& robot : *robots_) {
    positions.push_back(TypeConvertor::vector4dToEigen3d(robot.getPose()));
  }

  return positions;
}
//}

//}

/* VISUALIZATION //{ */

/* publishVisualizations() //{ */
void FormationReshapingNodelet::publishVisualizations(const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  if (!visualize_assignment_) {
    if (trajectories_set_ && run_visualization_) {
      for (auto& robot : *robots_) {
        robot.makeStep(visualization_real_time_factor_ / visualization_timer_rate_);
      }
    }
    publishRobots();
    publishTrajectories();
  } else {
    std::scoped_lock lock(mutex_comparison_visualizations_);
    publishCatora();
    publishLsap();
  }
}
//}

/* publishRobots()  //{ */
void FormationReshapingNodelet::publishRobots() {

  std::scoped_lock lock(mutex_robots_);
  bv_robots_.clearBuffers();
  for (size_t k = 0; k < robots_->size(); k++) {
    bv_robots_.addPoint(Eigen::Vector3d(robots_->at(k).state_.pos.x, robots_->at(k).state_.pos.y, robots_->at(k).state_.pos.z), 0, 0, 1, 1);
  }
  bv_robots_.publish();
}
//}

/* clearBvBuffers()  //{ */
void FormationReshapingNodelet::clearBvBuffers() {

  std::scoped_lock lock(mutex_bv_robots_);
  bv_robots_.clearBuffers();
  bv_ref_trajectories_.clearBuffers();
  bv_robots_.publish();
  bv_ref_trajectories_.publish();
}
//}

/* clearAssignmentVisualizations()  //{ */
void FormationReshapingNodelet::clearAssignmentVisualizations() {

  std::scoped_lock lock(mutex_comparison_visualizations_);

  visualize_assignment_ = false;
  start_points_.clear();
  end_points_.clear();
  assignment_catora_.clear();
  assignment_lsap_.clear();
  publishLsap();
  publishCatora();
}
//}

/* publishTrajectories() //{ */
void FormationReshapingNodelet::publishTrajectories() {

  std::scoped_lock lock(mutex_robots_, mutex_bv_ref_trajectories_);

  bv_ref_trajectories_.clearBuffers();
  double c_step = 1.0 / (double)robots_->size();
  for (size_t k = 0; k < robots_->size(); k++) {
    bv_ref_trajectories_.addTrajectory(TypeConvertor::trajectoryToMrsMsgsTrajectory(robots_->at(k).getTrajectory()), k * c_step, 0.8 - (k * c_step * 0.8), 0.0,
                                       1);
  }

  bv_ref_trajectories_.publish();
}
//}

/* publishLsap()  //{ */
void FormationReshapingNodelet::publishLsap() {

  std::scoped_lock lock(mutex_bv_lsap_);

  bv_lsap_.clearBuffers();
  for (auto& m : assignment_lsap_) {

    bv_lsap_.addPoint(start_points_[m.first], 0, 1, 0, 1);
    bv_lsap_.addPoint(end_points_[m.second], 1, 0, 0, 1);
    std::vector<Eigen::Vector3d> traj = {start_points_[m.first], end_points_[m.second]};
    bv_lsap_.addTrajectory(TypeConvertor::trajectoryToMrsMsgsTrajectory(traj), 0, 0, 1, 0.2);
  }

  bv_lsap_.publish();
}
//}

/* publishCatora()  //{ */
void FormationReshapingNodelet::publishCatora() {

  std::scoped_lock lock(mutex_bv_catora_);

  bv_catora_.clearBuffers();
  for (auto& m : assignment_catora_) {

    bv_catora_.addPoint(start_points_[m.first], 0, 1, 0, 1);
    bv_catora_.addPoint(end_points_[m.second], 1, 0, 0, 1);
    std::vector<Eigen::Vector3d> traj = {start_points_[m.first], end_points_[m.second]};
    bv_catora_.addTrajectory(TypeConvertor::trajectoryToMrsMsgsTrajectory(traj), 1, 0.5, 0.2, 0.2);
  }

  bv_catora_.publish();
}
//}

/* updateRobots() //{ */
void FormationReshapingNodelet::updateRobots(const std::vector<std::vector<Eigen::Vector3d>>& trajectories) {

  robots_->clear();
  for (size_t k = 0; k < trajectories.size(); k++) {  // initialize some robots, will be overwritten by additional requests
    RobotState state(trajectories.at(k).at(0)[0], trajectories.at(k).at(0)[1], trajectories.at(k).at(0)[2], 0.0);
    Robot      rob(state, k);
    robots_->push_back(rob);
  }
}

void FormationReshapingNodelet::updateRobots(int goal_number_of_robots) {

  if (robots_->size() > goal_number_of_robots) {
    robots_->erase(robots_->begin() + goal_number_of_robots, robots_->end());
  } else {  // the existing robot positions should be considered to avoid overlap in positions
    std::vector<Eigen::Vector3d> new_positions =
        getRandomVector(goal_number_of_robots - robots_->size(), test_area_horizontal_dimension_, test_area_vertical_dimension_, 1.0);
    int robot_idx = robots_->size();
    for (auto& position : new_positions) {
      RobotState state(position[0], position[1], position[2], 0.0);
      Robot      rob(state, ++robot_idx);
      robots_->push_back(rob);
    }
  }
}
//}

//}

}  // namespace mrs_formation_reshaping

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_formation_reshaping::FormationReshapingNodelet, nodelet::Nodelet);
