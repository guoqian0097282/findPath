#ifndef GRIDDEDGRAPH
#define GRIDDEDGRAPH

#include <memory>
#include <vector>

#include "obstacle.h"
#include "speed_data.h"
#include "st_point.h"
#include "st_graph_data.h"
#include "dp_st_cost.h"
#include "st_graph_point.h"


class GriddedPathTimeGraph
{
public:
  GriddedPathTimeGraph(const StGraphData &st_graph_data,
                       const DpStSpeedOptimizerConfig &dp_config,
                       const std::vector<const Obstacle *> &obstacles,
                       const TrajectoryPoint &init_point);

  bool Search(SpeedData *const speed_data);

private:
  double FLAGS_planning_upper_speed_limit = 2.7;
  bool InitCostTable();
  bool InitSpeedLimitLookUp();
  bool RetrieveSpeedProfile(SpeedData *const speed_data);
  bool CalculateTotalCost();

  // defined for cyber task
  struct StGraphMessage
  {
    StGraphMessage(const uint32_t c_, const int32_t r_) : c(c_), r(r_) {}
    uint32_t c;
    uint32_t r;
  };
  void CalculateCostAt(const std::shared_ptr<StGraphMessage> &msg);
  double CalculateEdgeCost(const STPoint &first, const STPoint &second,
                           const STPoint &third, const STPoint &forth,
                           const double speed_limit, const double cruise_speed);
  double CalculateEdgeCostForSecondCol(const uint32_t row,
                                       const double speed_limit,
                                       const double cruise_speed);
  double CalculateEdgeCostForThirdCol(const uint32_t curr_row,
                                      const uint32_t pre_row,
                                      const double speed_limit,
                                      const double cruise_speed);
  // get the row-range of next time step
  void GetRowRange(const StGraphPoint &point, size_t *next_highest_row,
                   size_t *next_lowest_row);

private:
  const StGraphData &st_graph_data_;
  std::vector<double> speed_limit_by_index_;
  std::vector<double> spatial_distance_by_index_;
  DpStSpeedOptimizerConfig gridded_path_time_graph_config_;
  const std::vector<const Obstacle *> &obstacles_;
  TrajectoryPoint init_point_;

  DpStCost dp_st_cost_;
  double total_length_t_ = 0.0;
  double unit_t_ = 0.0;
  uint32_t dimension_t_ = 0;
  double total_length_s_ = 0.0;
  double dense_unit_s_ = 0.0;
  double sparse_unit_s_ = 0.0;
  uint32_t dense_dimension_s_ = 0;
  uint32_t sparse_dimension_s_ = 0;
  uint32_t dimension_s_ = 0;
  double max_acceleration_ = 0.0;
  double max_deceleration_ = 0.0;

  // cost_table_[t][s]
  std::vector<std::vector<StGraphPoint>> cost_table_;
};
#endif