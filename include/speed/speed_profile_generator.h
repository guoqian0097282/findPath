#pragma once

#include <utility>
#include <vector>

// #include "modules/common/proto/pnc_point.pb.h"
// #include "modules/planning/common/ego_info.h"
// #include "modules/planning/common/reference_line_info.h"
#include "speed_data.h"
// #include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"


class SpeedProfileGenerator {
 public:
  SpeedProfileGenerator() = delete;

  static void FillEnoughSpeedPoints(SpeedData* const speed_data);

  static SpeedData GenerateFixedDistanceCreepProfile(const double distance,
                                                     const double max_speed);

 private:
//   static SpeedData GenerateStopProfile(const double init_speed,
//                                        const double init_acc);
};
