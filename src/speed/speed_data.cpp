#include "speed_data.h"

#include <algorithm>
#include <mutex>
#include <utility>

// #include "absl/strings/str_cat.h"
// #include "absl/strings/str_join.h"
#include "linear_interpolation.h"
// #include "modules/common/util/point_factory.h"
#include "util.h"
// #include "planning_gflags.h"


SpeedData::SpeedData(std::vector<SpeedPoint> speed_points)
    : std::vector<SpeedPoint>(std::move(speed_points)) {
  std::sort(begin(), end(), [](const SpeedPoint& p1, const SpeedPoint& p2) {
    return p1.t < p2.t;
  });
}

void SpeedData::AppendSpeedPoint(const double s, const double time,
                                 const double v, const double a,
                                 const double da) {
  static std::mutex mutex_speedpoint;
  // UNIQUE_LOCK_MULTITHREAD(mutex_speedpoint);

  // if (!empty()) {
    // ACHECK(back().t() < time);
  // }
  SpeedPoint speedPoint;
  speedPoint.s = s;
  speedPoint.t = time;
  speedPoint.v = v;
  speedPoint.a = a;
  speedPoint.da = da;
  push_back(speedPoint);
}



bool SpeedData::EvaluateByTime(const double t, SpeedPoint* const speed_point) const {
  if (size() < 2) {
    return false;
  }
  // 越界，则返回 false
  if (!(front().t < t + 1.0e-6 && t - 1.0e-6 < back().t)) {
    return false;
  }

  auto comp = [](const SpeedPoint& sp, const double t) {
    return sp.t < t;
  };

  auto it_lower = std::lower_bound(begin(), end(), t, comp);
  if (it_lower == end()) {
    *speed_point = back();
  } else if (it_lower == begin()) {
    *speed_point = front();
  } else {
    const auto& p0 = *(it_lower - 1);
    const auto& p1 = *it_lower;
    double t0 = p0.t;
    double t1 = p1.t;

    // speed_point->Clear();
    // 使用t，在p0和p1之间线性插值
    speed_point->s = lerp(p0.s, t0, p1.s, t1, t);
    speed_point->t = t;
    if (p0.v && p1.v) {
      speed_point->v = lerp(p0.v, t0, p1.v, t1, t);
    }
    if (p0.a && p1.a) {
      speed_point->a = lerp(p0.a, t0, p1.a, t1, t);
    }
    if (p0.da && p1.da) {
      speed_point->da = lerp(p0.da, t0, p1.da, t1, t);
    }
  }
  return true;
}

bool SpeedData::EvaluateByS(const double s,
                            SpeedPoint* const speed_point) const {
  if (size() < 2) {
    return false;
  }
  if (!(front().s < s + 1.0e-6 && s - 1.0e-6 < back().s)) {
    return false;
  }

  auto comp = [](const SpeedPoint& sp, const double s) {
    return sp.s < s;
  };

  auto it_lower = std::lower_bound(begin(), end(), s, comp);
  if (it_lower == end()) {
    *speed_point = back();
  } else if (it_lower == begin()) {
    *speed_point = front();
  } else {
    const auto& p0 = *(it_lower - 1);
    const auto& p1 = *it_lower;
    double s0 = p0.s;
    double s1 = p1.s;

    // speed_point->Clear();
    speed_point->s = s;
    speed_point->t = lerp(p0.t, s0, p1.t, s1, s);
    if (p0.v && p1.v) {
      speed_point->v = lerp(p0.v, s0, p1.v, s1, s);
    }
    if (p0.a && p1.a) {
      speed_point->a = lerp(p0.a, s0, p1.a, s1, s);
    }
    if (p0.da && p1.da) {
      speed_point->da = lerp(p0.da, s0, p1.da, s1, s);
    }
  }
  return true;
}

double SpeedData::TotalTime() const {
  if (empty()) {
    return 0.0;
  }
  return back().t - front().t;
}

double SpeedData::TotalLength() const {
  if (empty()) {
    return 0.0;
  }
  return back().s - front().s;
}