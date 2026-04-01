
#pragma once

#include <tuple>
#include <vector>

#include "pnc_point.h"
#include "st_boundary.h"
#include "speed_limit.h"
struct STDrivableBoundaryInstance
{
  double t = 1;
  double s_lower = 2;
  double s_upper = 3;
  double v_obs_lower = 4;
  double v_obs_upper = 5;
};

struct STDrivableBoundary
{
  std::vector<STDrivableBoundaryInstance> st_boundary;
};
constexpr double kObsSpeedIgnoreThreshold = 100.0;

class StGraphData
{
public:
  StGraphData() = default;

  void LoadData(const std::vector<const STBoundary *> &st_boundaries,
                const double min_s_on_st_boundaries,
                const TrajectoryPoint &init_point,
                const SpeedLimit &speed_limit, const double cruise_speed,
                const double path_data_length, const double total_time_by_conf);

  bool is_initialized() const { return init_; }

  const std::vector<const STBoundary *> &st_boundaries() const;

  double min_s_on_st_boundaries() const;

  const TrajectoryPoint &init_point() const;

  const SpeedLimit &speed_limit() const;

  double cruise_speed() const;

  double path_length() const;

  double total_time_by_conf() const;

  // bool SetSTDrivableBoundary(
  //     const std::vector<std::tuple<double, double, double>> &s_boundary,
  //     const std::vector<std::tuple<double, double, double>> &v_obs_info);

  const STDrivableBoundary &st_drivable_boundary() const;

private:
  bool init_ = false;
  std::vector<const STBoundary *> st_boundaries_;
  double min_s_on_st_boundaries_ = 0.0;
  TrajectoryPoint init_point_;
  SpeedLimit speed_limit_;
  double cruise_speed_ = 0.0;
  double path_data_length_ = 0.0;
  double path_length_by_conf_ = 0.0;
  double total_time_by_conf_ = 0.0;

  STDrivableBoundary st_drivable_boundary_;
};