#ifndef DISPATH
#define DISPATH

#include <utility>
#include <vector>

#include "pnc_point.h"

class DiscretizedPath : public std::vector<PathPoint> {
 public:
  DiscretizedPath() = default;

  explicit DiscretizedPath(std::vector<PathPoint> path_points);

  double Length() const;

  PathPoint Evaluate(const double path_s) const;

  PathPoint EvaluateReverse(const double path_s) const;

 protected:
  std::vector<PathPoint>::const_iterator QueryLowerBound(
      const double path_s) const;
  std::vector<PathPoint>::const_iterator QueryUpperBound(
      const double path_s) const;
};

#endif