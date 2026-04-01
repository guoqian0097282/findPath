#ifndef FREPATH
#define FREPATH

#include <utility>
#include <vector>

#include "pnc_point.h"


class FrenetFramePath : public std::vector<FrenetFramePoint> {
 public:
  FrenetFramePath() = default;
  explicit FrenetFramePath(std::vector<FrenetFramePoint> points);

  double Length() const;
  FrenetFramePoint EvaluateByS(const double s) const;

  /**
   * @brief Get the FrenetFramePoint that is within SLBoundary, or the one with
   * smallest l() in SLBoundary's s range [start_s(), end_s()]
   */
  FrenetFramePoint GetNearestPoint(const SLBoundary &sl) const;

 private:
  static bool LowerBoundComparator(const FrenetFramePoint &p,
                                   const double s) {
    return p.s < s;
  }
  static bool UpperBoundComparator(const double s,
                                   const FrenetFramePoint &p) {
    return s < p.s;
  }
};

#endif