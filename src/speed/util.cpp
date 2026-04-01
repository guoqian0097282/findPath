#include "util.h"

#include <cmath>
#include <vector>

PointENU operator+(const PointENU enu, const Vec2d& xy) {
  PointENU point;
  point.x = enu.x + xy.x();
  point.y = enu.y + xy.y();
  point.z = enu.z;
  return point;
}

PathPoint GetWeightedAverageOfTwoPathPoints(const PathPoint& p1,
                                            const PathPoint& p2,
                                            const double w1, const double w2) {
  PathPoint p;
  p.x = p1.x * w1 + p2.x * w2;
  p.y = p1.y * w1 + p2.y * w2;
  p.z = p1.z * w1 + p2.z * w2;
  p.theta = p1.theta * w1 + p2.theta * w2;
  p.kappa = p1.kappa * w1 + p2.kappa * w2;
  p.dkappa = p1.dkappa * w1 + p2.dkappa * w2;
  p.ddkappa = p1.ddkappa * w1 + p2.ddkappa * w2;
  p.s = p1.s * w1 + p2.s * w2;
  return p;
}