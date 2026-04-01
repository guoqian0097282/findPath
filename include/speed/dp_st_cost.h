#ifndef STCOST
#define STCOST
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "pnc_point.h"
#include "obstacle.h"
#include "st_boundary.h"
#include "st_point.h"
#include "st_graph_point.h"
#include "st_graph_data.h"
struct DpStSpeedOptimizerConfig
{
  double unit_t;
  int dense_dimension_s;
  double dense_unit_s;
  double sparse_unit_s;

  double speed_weight;
  double accel_weight;
  double jerk_weight;
  double obstacle_weight;
  double reference_weight;
  double go_down_buffer;
  double go_up_buffer;

  // obstacle cost config
  double default_obstacle_cost;

  // speed cost config
  double default_speed_cost;
  double exceed_speed_penalty;
  double low_speed_penalty;
  double reference_speed_penalty;
  double keep_clear_low_speed_penalty;

  // accel cost config
  double accel_penalty;
  double decel_penalty;

  // jerk cost config
  double positive_jerk_coeff;
  double negative_jerk_coeff;

  // other constraint
  double max_acceleration;
  double max_deceleration;

  // buffer
  double safe_time_buffer;
  double safe_distance;

  // spatial potential cost config for minimal time traversal
  double spatial_potential_penalty;

  bool is_lane_changing;
};

class DpStCost
{
public:
  DpStCost(const DpStSpeedOptimizerConfig &config, const double total_t,
           const double total_s, const std::vector<const Obstacle *> &obstacles,
           const STDrivableBoundary &st_drivable_boundary,
           const TrajectoryPoint &init_point);

  double GetObstacleCost(const StGraphPoint &point);

  double GetSpatialPotentialCost(const StGraphPoint &point);

  double GetReferenceCost(const STPoint &point,
                          const STPoint &reference_point) const;

  double GetSpeedCost(const STPoint &first, const STPoint &second,
                      const double speed_limit,
                      const double cruise_speed) const;

  double GetAccelCostByTwoPoints(const double pre_speed, const STPoint &first,
                                 const STPoint &second);
  double GetAccelCostByThreePoints(const STPoint &first, const STPoint &second,
                                   const STPoint &third);

  double GetJerkCostByTwoPoints(const double pre_speed, const double pre_acc,
                                const STPoint &pre_point,
                                const STPoint &curr_point);
  double GetJerkCostByThreePoints(const double first_speed,
                                  const STPoint &first_point,
                                  const STPoint &second_point,
                                  const STPoint &third_point);
  double GetJerkCostByFourPoints(const STPoint &first, const STPoint &second,
                                 const STPoint &third, const STPoint &fourth);

private:
  bool FLAGS_use_st_drivable_boundary = false;
  bool FLAGS_enable_dp_reference_speed = true;
  double FLAGS_speed_lon_decision_horizon = 200;
  double GetAccelCost(const double accel);
  double JerkCost(const double jerk);

  void AddToKeepClearRange(const std::vector<const Obstacle *> &obstacles);
  static void SortAndMergeRange(std::vector<std::pair<double, double>> *keep_clear_range_);
  bool InKeepClearRange(double s) const;

  const DpStSpeedOptimizerConfig &config_;
  const std::vector<const Obstacle *> &obstacles_;
  STDrivableBoundary st_drivable_boundary_;
  const TrajectoryPoint &init_point_;
  double unit_t_ = 0.0;
  double total_s_ = 0.0;

  std::unordered_map<std::string, int> boundary_map_;
  std::vector<std::vector<std::pair<double, double>>> boundary_cost_;
  std::vector<std::pair<double, double>> keep_clear_range_;

  std::array<double, 200> accel_cost_;
  std::array<double, 400> jerk_cost_;
};
#endif