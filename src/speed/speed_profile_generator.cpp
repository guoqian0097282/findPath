#include "speed_profile_generator.h"

#include <algorithm>
#include <utility>

// #include "log.h"
// #include "modules/planning/common/frame.h"
// #include "planning_gflags.h"
#include "piecewise_jerk_speed_problem.h"

// #include "pnc_point.h"


void SpeedProfileGenerator::FillEnoughSpeedPoints(SpeedData* const speed_data) {
  const SpeedPoint& last_point = speed_data->back();
  if (last_point.t >= 30) {
    return;
  }
  for (double t = last_point.t + 0.1;t < 30; t += 0.1) {
    speed_data->AppendSpeedPoint(last_point.s, t, 0.0, 0.0, 0.0);
  }
}

// SpeedData SpeedProfileGenerator::GenerateStopProfile(const double init_speed,
//                                                      const double init_acc) {
//   // AERROR << "Slowing down the car within a constant deceleration with fallback "
//   //           "stopping profile.";
//   SpeedData speed_data;

//   const double max_t = FLAGS_fallback_total_time;
//   const double unit_t = FLAGS_fallback_time_unit;

//   double pre_s = 0.0;
//   double pre_v = init_speed;
//   double acc = FLAGS_slowdown_profile_deceleration;

//   speed_data.AppendSpeedPoint(0.0, 0.0, init_speed, init_acc, 0.0);
//   for (double t = unit_t; t < max_t; t += unit_t) {
//     double s = 0.0;
//     double v = 0.0;
//     s = std::fmax(pre_s,
//                   pre_s + 0.5 * (pre_v + (pre_v + unit_t * acc)) * unit_t);
//     v = std::fmax(0.0, pre_v + unit_t * acc);
//     speed_data.AppendSpeedPoint(s, t, v, acc, 0.0);
//     pre_s = s;
//     pre_v = v;
//   }
//   FillEnoughSpeedPoints(&speed_data);
//   return speed_data;
// }

SpeedData SpeedProfileGenerator::GenerateFixedDistanceCreepProfile(
    const double distance, const double max_speed) {
  static constexpr double kConstDeceleration = -0.8;  // (~3sec to fully stop)
  static constexpr double kProceedingSpeed = 2.23;    // (5mph proceeding speed)
  const double proceeding_speed = std::min(max_speed, kProceedingSpeed);
  const double distance_to_start_deceleration =
      proceeding_speed * proceeding_speed / kConstDeceleration / 2;
  bool is_const_deceleration_mode = distance < distance_to_start_deceleration;

  double a = kConstDeceleration;
  double t = 0.0;
  double s = 0.0;
  double v = proceeding_speed;

  static constexpr double kDeltaT = 0.1;

  SpeedData speed_data;
  while (s < distance && v > 0) {
    if (is_const_deceleration_mode) {
      speed_data.AppendSpeedPoint(s, t, v, a, 0.0);
      t += kDeltaT;
      double v_new = std::max(0.0, v + a * t);
      s += kDeltaT * (v + v_new) / 2;
      v = v_new;
    } else {
      speed_data.AppendSpeedPoint(s, t, v, 0.0, 0.0);
      t += kDeltaT;
      s += kDeltaT * v;
      if (distance - s < distance_to_start_deceleration)
        is_const_deceleration_mode = true;
    }
  }

  return speed_data;
}