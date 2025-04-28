#include "mrs_formation_reshaping/formation_reshaper.hpp"
#include "mrs_formation_reshaping/catora.hpp"


using namespace mrs_formation_reshaping;
using namespace Catora;

/* FormationReshaper //{ */

FormationReshaper::FormationReshaper(void) {
}

FormationReshaper::FormationReshaper(double max_acc, double max_vel, double trajectory_dt) {
  max_acc_       = max_acc;
  max_vel_       = max_vel;
  trajectory_dt_ = trajectory_dt;
}

FormationReshaper::~FormationReshaper(void) {
}

//}

/* getReshapingTrajectoriesCatora() //{ */
std::vector<std::vector<Eigen::Vector3d>> FormationReshaper::getReshapingTrajectoriesCatora(const std::vector<Eigen::Vector3d> &initial_configurations,
                                                                                            const std::vector<Eigen::Vector3d> &goal_configurations) {
  return getReshapingTrajectories(initial_configurations, goal_configurations, true);
}
//}

/* getReshapingTrajectoriesLsap() //{ */
std::vector<std::vector<Eigen::Vector3d>> FormationReshaper::getReshapingTrajectoriesLsap(const std::vector<Eigen::Vector3d> &initial_configurations,
                                                                                          const std::vector<Eigen::Vector3d> &goal_configurations) {
  return getReshapingTrajectories(initial_configurations, goal_configurations, false);
}
//}

/* getReshapingTrajectories() //{ */
std::vector<std::vector<Eigen::Vector3d>> FormationReshaper::getReshapingTrajectories(const std::vector<Eigen::Vector3d> &initial_configurations,
                                                                                      const std::vector<Eigen::Vector3d> &goal_configurations,
                                                                                      bool                                use_catora) {

  std::vector<std::vector<Eigen::Vector3d>> trajectories;

  if (initial_configurations.empty() || goal_configurations.empty()) {
    ROS_WARN("[%s]: Reshaping trajectories cannot be generated for empty set of configurations.", ros::this_node::getName().c_str());
    return trajectories;
  }

  if (initial_configurations.size() != goal_configurations.size()) {
    ROS_WARN("[%s]: Number of initial configurations (%lu) does not match goal configurations (%lu). Reshaping trajectories cannot be generated.",
             ros::this_node::getName().c_str(), initial_configurations.size(), goal_configurations.size());
    return trajectories;
  }

  std::pair<std::vector<std::pair<int, int>>, std::vector<double>> res;

  if (use_catora) {
    res = getCatoraAssignment(initial_configurations, goal_configurations);
  } else {
    res = getLsapAssignment(initial_configurations, goal_configurations);
  }

  auto assignment = res.first;

  double trajectory_duration;

  auto start                                  = std::chrono::high_resolution_clock::now();
  std::tie(trajectories, trajectory_duration) = generateTrajectoriesMinTime(initial_configurations, goal_configurations, assignment);
  std::chrono::duration<double> diff          = std::chrono::high_resolution_clock::now() - start;

  double comp_time_total = res.second[0] + diff.count() * 1000.0;  // time in ms

  ROS_INFO("[MrsFormationReshaper]: Reshaping trajectories generated: trajectory duration = %.2f s, computational time = %.2f ms",
           trajectories[0].size() * trajectory_dt_, comp_time_total);

  return trajectories;
}
//}

/* getCatoraAssignment() //{ */
std::pair<std::vector<std::pair<int, int>>, std::vector<double>> FormationReshaper::getCatoraAssignment(const std::vector<Eigen::Vector3d> &robot_positions,
                                                                                                        const std::vector<Eigen::Vector3d> &formation_goals) {

  auto start_time = std::chrono::high_resolution_clock::now();

  std::vector<std::pair<int, int>> indices_mapping;

  std::vector<std::vector<int>> distances = getDistanceMatrixEuclidean(robot_positions, formation_goals);

  std::vector<std::vector<int>> assignment;

  std::vector<double> solution_times_ms(2);

  double min_init_dist = getMinInitialDist(robot_positions, formation_goals);

  std::vector<std::vector<int>> starts_dists = getDistanceMatrixEuclidean(robot_positions, robot_positions);
  std::vector<std::vector<int>> goals_dists  = getDistanceMatrixEuclidean(formation_goals, formation_goals);
  double                        min_dist     = 1 / sqrt(2) * min_init_dist;

  ROS_INFO("[%s]: Starting CAT-ORA assignment algorithm.", ros::this_node::getName().c_str());
  auto res = catora_assignment(distances, starts_dists, goals_dists, robot_positions, formation_goals, min_dist);

  assignment           = std::get<1>(res);
  solution_times_ms[1] = std::get<2>(res);

  ROS_INFO("[%s]: CATORA assignment: cost = %d, comp. time = %.3f ms", ros::this_node::getName().c_str(), std::get<0>(res), solution_times_ms[1]);

  std::chrono::duration<double> diff = std::chrono::high_resolution_clock::now() - start_time;
  solution_times_ms[0]               = diff.count() * 1000.0;
  ;

  for (int row_from = 0; row_from < assignment.size(); row_from++) {
    int col_to = getIndexWithOne(assignment[row_from]);
    indices_mapping.push_back(std::pair(col_to, row_from));
  }

  return std::make_pair(indices_mapping, solution_times_ms);
}
//}

/* getLsapAssignment() //{ */
std::pair<std::vector<std::pair<int, int>>, std::vector<double>> FormationReshaper::getLsapAssignment(const std::vector<Eigen::Vector3d> &robot_positions,
                                                                                                      const std::vector<Eigen::Vector3d> &formation_goals) {

  auto start_time = std::chrono::high_resolution_clock::now();

  std::vector<std::pair<int, int>> indices_mapping;

  std::vector<std::vector<int>> distances = getDistanceMatrixEuclidean(robot_positions, formation_goals);

  std::vector<std::vector<int>> assignment;

  std::vector<double> solution_times_ms(2);

  double min_init_dist = getMinInitialDist(robot_positions, formation_goals);

  ROS_INFO("[%s]: Starting original Hungarian algorithm.", ros::this_node::getName().c_str());
  auto res = hungarian(distances);

  assignment           = std::get<1>(res);
  solution_times_ms[1] = std::get<2>(res);

  std::chrono::duration<double> diff = std::chrono::high_resolution_clock::now() - start_time;
  solution_times_ms[0]               = diff.count() * 1000.0;
  ;

  ROS_INFO("[%s]: LSAP solution: cost = %d, comp. time = %.3f ms", ros::this_node::getName().c_str(), std::get<0>(res), solution_times_ms[1]);

  // extract mapping of indices from assignment
  for (int row_from = 0; row_from < assignment.size(); row_from++) {
    int col_to = getIndexWithOne(assignment[row_from]);
    indices_mapping.push_back(std::pair(col_to, row_from));
  }

  return std::make_pair(indices_mapping, solution_times_ms);
}
//}

/* generateTrajectoriesMinTime() //{ */
std::pair<std::vector<std::vector<Eigen::Vector3d>>, double> FormationReshaper::generateTrajectoriesMinTime(const std::vector<Eigen::Vector3d> &    starts,
                                                                                                            const std::vector<Eigen::Vector3d> &    goals,
                                                                                                            const std::vector<std::pair<int, int>> &f_mapping) {

  std::vector<std::vector<Eigen::Vector3d>> trajectories;

  // get maximum path length
  double dist_max = 0.0;
  for (auto &fm : f_mapping) {

    double dist = (starts[fm.first] - goals[fm.second]).norm();
    if (dist > dist_max) {
      dist_max = dist;
    }
  }

  double time_to_max_v = max_vel_ / max_acc_;
  double dist_to_max_v = 0.5 * max_vel_ * time_to_max_v;
  double max_vel_applied = max_vel_;
  if (dist_to_max_v > dist_max / 2.0) {
    time_to_max_v = sqrt(dist_max / max_acc_);
    max_vel_applied = time_to_max_v * max_acc_;
    dist_to_max_v = dist_max / 2.0;
  }
  double time_total = max_vel_applied > 1e-3 ? 2 * time_to_max_v + (dist_max - 2 * dist_to_max_v) / max_vel_applied : 0.0;

  trajectories.resize(starts.size());
  for (size_t k = 0; k < starts.size(); k++) {
    trajectories[f_mapping[k].first] =
        getMinimumTimeTrajectory(starts[f_mapping[k].first], goals[f_mapping[k].second], time_total, trajectory_dt_, max_acc_, max_vel_applied, dist_max);
  }

  return std::make_pair(trajectories, time_total);
}
//}

/* getMinimumTimeTrajectory() //{ */
std::vector<Eigen::Vector3d> FormationReshaper::getMinimumTimeTrajectory(Eigen::Vector3d start, Eigen::Vector3d goal, double time_total, double ts, double am,
                                                                         double vm, double dm) {

  std::vector<Eigen::Vector3d> trajectory;
  
  if (time_total < 1e-3) { // handles case when the start and goal coincides
    trajectory.push_back(start);
  } else { 

    double                       t_acc   = am < 1e-3 ? 0 : vm / am;
    int                          n_steps = ceil(time_total / ts);

    for (int k = 0; k <= n_steps; k++) {

      double s = std::min(k * ts / time_total, 1.0);
      double coeff;
      if (s <= t_acc / time_total) {
        coeff = am * pow(time_total, 2) * pow(s, 2) / (2 * dm);
      } else if (s <= (time_total - t_acc) / time_total) {
        coeff = vm * time_total * s / dm - pow(vm, 2) / (2 * am * dm);
      } else {
        coeff = am * pow(time_total, 2) * (2 * s - pow(s, 2)) / (2 * dm) - (pow(vm, 4) + pow(am, 2) * pow(dm, 2)) / (2 * am * dm * pow(vm, 2));
      }

      Eigen::Vector3d point = start + coeff * (goal - start);
      trajectory.push_back(point);
    }

  }

  return trajectory;
}
//}

/* getDistanceMatrixEuclidean() //{ */
std::vector<std::vector<int>> FormationReshaper::getDistanceMatrixEuclidean(const std::vector<Eigen::Vector3d> &start_points,
                                                                            const std::vector<Eigen::Vector3d> &goal_points) {

  std::vector<std::vector<int>> distances;

  for (size_t f = 0; f < goal_points.size(); f++) {

    std::vector<int> l_dist;

    for (size_t r = 0; r < start_points.size(); r++) {
      l_dist.push_back((int)SCALE_FACTOR * pow((goal_points[f] - start_points[r]).norm(), 2));
    }

    distances.push_back(l_dist);
  }

  return distances;
}
//}

/* getMinInitialDist() //{ */
double FormationReshaper::getMinInitialDist(const std::vector<Eigen::Vector3d> &initial_positions, const std::vector<Eigen::Vector3d> &final_positions) {

  double min_init_dist = 1e6;
  for (size_t s = 0; s < initial_positions.size(); s++) {
    for (size_t g = s + 1; g < initial_positions.size(); g++) {
      double dist = (initial_positions[s] - initial_positions[g]).norm();
      if (min_init_dist > dist) {
        min_init_dist = dist;
      }
    }
  }

  for (size_t s = 0; s < final_positions.size(); s++) {
    for (size_t g = s + 1; g < final_positions.size(); g++) {
      double dist = (final_positions[s] - final_positions[g]).norm();
      if (min_init_dist > dist) {
        min_init_dist = dist;
      }
    }
  }

  return min_init_dist;
}
//}

/* getIndexWithOne() //{ */
int FormationReshaper::getIndexWithOne(const std::vector<int> &v) {
  int index = -1;

  auto it = std::find(v.begin(), v.end(), 1);
  if (it != v.end()) {
    index = std::distance(v.begin(), it);
  }

  return index;
}
//}

/* setConstraintsAndDt() //{ */
void FormationReshaper::setConstraintsAndDt(double max_acc, double max_vel, double trajectory_dt) {
  max_acc_       = max_acc;
  max_vel_       = max_vel;
  trajectory_dt_ = trajectory_dt;
}
//}
