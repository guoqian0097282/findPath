#include "obstacle.h"

#include <algorithm>
#include <utility>

#include "linear_interpolation.h"
// #include "modules/common/util/map_util.h"
#include "util.h"
#include "planning_gflags.h"
#include "st_boundary.h"
#include "decision.h"
#include "pnc_point.h"
#include "prediction_obstacle.h"

namespace
{
  const double kStBoundaryDeltaS = 0.2;       // meters
  const double kStBoundarySparseDeltaS = 1.0; // meters
  const double kStBoundaryDeltaT = 0.05;      // seconds
} // namespace
bool FLAGS_use_navigation_mode = false;
// const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
//                          Obstacle::ObjectTagCaseHash>
//     Obstacle::s_longitudinal_decision_safety_sorter_ = {
//         {ObjectDecisionType::IGNORE},
//         {ObjectDecisionType::OVERTAKE},
//         {ObjectDecisionType::FOLLOW},
//         {ObjectDecisionType::YIELD},
//         {ObjectDecisionType::STOP}};

const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                         Obstacle::ObjectTagCaseHash>
    Obstacle::s_lateral_decision_safety_sorter_ = {
        {ObjectDecisionType::IGNORE, 0}, {ObjectDecisionType::NUDGE, 100}};

Obstacle::Obstacle(const std::string &id,
                   const PerceptionObstacle &perception_obstacle,
                   const ObstaclePriority::Priority &obstacle_priority,
                   const bool is_static)
    : id_(id),
      perception_id_(perception_obstacle.id),
      perception_obstacle_(perception_obstacle),
      perception_bounding_box_({perception_obstacle_.position.x,
                                perception_obstacle_.position.y},
                               perception_obstacle_.theta,
                               perception_obstacle_.length,
                               perception_obstacle_.width)
{
  is_caution_level_obstacle_ = (obstacle_priority == ObstaclePriority::CAUTION);
  std::vector<Vec2d> polygon_points;
  if (FLAGS_use_navigation_mode || perception_obstacle.polygon_point_size <= 2)
  {
    perception_bounding_box_.GetAllCorners(&polygon_points);
  }
  else
  {
    if (perception_obstacle.polygon_point_size > 2)
    {
      std::cout << "object " << id << "has less than 3 polygon points";
    }
    for (const auto &point : perception_obstacle.polygon_point)
    {
      polygon_points.emplace_back(point.x, point.y);
    }
  }
  if (Polygon2d::ComputeConvexHull(polygon_points, &perception_polygon_))
  {
    std::cout << "object[" << id << "] polygon is not a valid convex hull.\n";
  }
  is_static_ = (is_static || obstacle_priority == ObstaclePriority::IGNORE);
  is_virtual_ = (perception_obstacle.id < 0);
  speed_ = std::hypot(perception_obstacle.velocity.x,
                      perception_obstacle.velocity.y);
}

Obstacle::Obstacle(const std::string &id,
                   const PerceptionObstacle &perception_obstacle,
                   const TrajectoryObstacle &trajectory,
                   const ObstaclePriority::Priority &obstacle_priority,
                   const bool is_static)
    : Obstacle(id, perception_obstacle, obstacle_priority, is_static)
{
  trajectory_ = trajectory;
  auto &trajectory_points = trajectory_.trajectory_point;
  double cumulative_s = 0.0;
  if (trajectory_points.size() > 0)
  {
    trajectory_points[0].path_point.s = 0.0;
  }
  for (int i = 1; i < trajectory_points.size(); ++i)
  {
    const auto &prev = trajectory_points[i - 1];
    const auto &cur = trajectory_points[i];
    if (prev.relative_time >= cur.relative_time)
    {
      std::cout << "prediction time is not increasing.";
    }
    cumulative_s +=
        DistanceXY(prev.path_point, cur.path_point);
    trajectory_points[i].path_point.s = cumulative_s;
  }
}

TrajectoryPoint Obstacle::GetPointAtTime(
    const double relative_time) const
{
  const auto &points = trajectory_.trajectory_point;
  if (points.size() < 2)
  {
    TrajectoryPoint point;
    point.path_point.x = perception_obstacle_.position.x;
    point.path_point.y = perception_obstacle_.position.y;
    point.path_point.z = perception_obstacle_.position.z;
    point.path_point.theta = perception_obstacle_.theta;
    point.path_point.s = 0.0;
    point.path_point.kappa = 0.0;
    point.path_point.dkappa = 0.0;
    point.path_point.ddkappa = 0.0;
    point.v = 0.0;
    point.a = 0.0;
    point.relative_time = 0.0;
    return point;
  }
  else
  {
    auto comp = [](const TrajectoryPoint p, const double time)
    {
      return p.relative_time < time;
    };

    auto it_lower =
        std::lower_bound(points.begin(), points.end(), relative_time, comp);

    if (it_lower == points.begin())
    {
      return *points.begin();
    }
    else if (it_lower == points.end())
    {
      return *points.rbegin();
    }
    return InterpolateUsingLinearApproximation(
        *(it_lower - 1), *it_lower, relative_time);
  }
}

Box2d Obstacle::GetBoundingBox(
    const TrajectoryPoint &point) const
{
  return Box2d({point.path_point.x, point.path_point.y},
               point.path_point.theta,
               perception_obstacle_.length,
               perception_obstacle_.width);
}

bool Obstacle::IsValidPerceptionObstacle(const PerceptionObstacle &obstacle)
{
  if (obstacle.length <= 0.0)
  {
    std::cout << "invalid obstacle length:" << obstacle.length;
    return false;
  }
  if (obstacle.width <= 0.0)
  {
    std::cout << "invalid obstacle width:" << obstacle.width;
    return false;
  }
  if (obstacle.height <= 0.0)
  {
    std::cout << "invalid obstacle height:" << obstacle.height;
    return false;
  }
  if (obstacle.velocity.x == 0)
  {
    if (std::isnan(obstacle.velocity.x) ||
        std::isnan(obstacle.velocity.y))
    {
      std::cout << "invalid obstacle velocity:";
      return false;
    }
  }
  for (auto pt : obstacle.polygon_point)
  {
    if (std::isnan(pt.x) || std::isnan(pt.y))
    {
      std::cout << "invalid obstacle polygon point:";
      return false;
    }
  }
  return true;
}

std::list<std::unique_ptr<Obstacle>> Obstacle::CreateObstacles(
    const PredictionObstacles &predictions)
{
  std::list<std::unique_ptr<Obstacle>> obstacles;
  for (const auto &prediction_obstacle : predictions.prediction_obstacle)
  {
    if (!IsValidPerceptionObstacle(prediction_obstacle.perception_obstacle))
    {
      std::cout << "Invalid perception obstacle: ";
      continue;
    }
    const auto perception_id =
        std::to_string(prediction_obstacle.perception_obstacle.id);
    if (prediction_obstacle.trajectory.size() == 0)
    {
      obstacles.emplace_back(
          new Obstacle(perception_id, prediction_obstacle.perception_obstacle,
                       prediction_obstacle.priority.priority,
                       prediction_obstacle.is_static));
      continue;
    }

    int trajectory_index = 0;
    for (const auto &trajectory : prediction_obstacle.trajectory)
    {
      bool is_valid_trajectory = true;
      for (const auto &point : trajectory.trajectory_point)
      {
        if (!IsValidTrajectoryPoint(point))
        {
          std::cout << "obj:" << perception_id
                    << " TrajectoryPoint: " << " is NOT valid.";
          is_valid_trajectory = false;
          break;
        }
      }
      if (!is_valid_trajectory)
      {
        continue;
      }

      const std::string obstacle_id = perception_id;
      // absl::StrCat(perception_id, "_", trajectory_index);
      obstacles.emplace_back(
          new Obstacle(obstacle_id, prediction_obstacle.perception_obstacle,
                       trajectory, prediction_obstacle.priority.priority,
                       prediction_obstacle.is_static));
      ++trajectory_index;
    }
  }
  return obstacles;
}

std::unique_ptr<Obstacle> Obstacle::CreateStaticVirtualObstacles(
    const std::string &id, const Box2d &obstacle_box)
{
  // create a "virtual" perception_obstacle
  PerceptionObstacle perception_obstacle;
  // simulator needs a valid integer
  size_t negative_id = std::hash<std::string>{}(id);
  // set the first bit to 1 so negative_id became negative number
  negative_id |= (0x1 << 31);
  perception_obstacle.id = static_cast<int32_t>(negative_id);
  perception_obstacle.position.x = obstacle_box.center().x();
  perception_obstacle.position.y = obstacle_box.center().y();
  perception_obstacle.theta = obstacle_box.heading();
  perception_obstacle.velocity.x = 0;
  perception_obstacle.velocity.y = 0;
  perception_obstacle.length = obstacle_box.length();
  perception_obstacle.width = obstacle_box.width();
  perception_obstacle.height = 2.0; // FLAGS_virtual_stop_wall_height;
  perception_obstacle.type = PerceptionObstacle::UNKNOWN_UNMOVABLE;
  perception_obstacle.tracking_time = 1.0;

  std::vector<Vec2d> corner_points;

  obstacle_box.GetAllCorners(&corner_points);
  int i = 0;
  perception_obstacle.polygon_point_size = corner_points.size();
  std::vector<Point3D> polygon_points;
  for (const auto &corner_point : corner_points)
  {
    Point3D point;
    point.x = corner_point.x();
    point.y = corner_point.y();
    polygon_points.push_back(point);
    i++;
  }
  perception_obstacle.polygon_point = polygon_points;
  auto *obstacle =
      new Obstacle(id, perception_obstacle, ObstaclePriority::NORMAL, true);
  obstacle->is_virtual_ = true;
  return std::unique_ptr<Obstacle>(obstacle);
}

bool Obstacle::IsValidTrajectoryPoint(const TrajectoryPoint &point)
{
  return !(std::isnan(point.path_point.x) ||
           std::isnan(point.path_point.y) ||
           std::isnan(point.path_point.z) ||
           std::isnan(point.path_point.kappa) ||
           std::isnan(point.path_point.s) ||
           std::isnan(point.path_point.dkappa) ||
           std::isnan(point.path_point.ddkappa) || std::isnan(point.v) ||
           std::isnan(point.a) || std::isnan(point.relative_time));
}

void Obstacle::SetPerceptionSlBoundary(const SLBoundary &sl_boundary)
{
  sl_boundary_ = sl_boundary;
}

const STBoundary &Obstacle::reference_line_st_boundary() const
{
  return reference_line_st_boundary_;
}

const STBoundary &Obstacle::path_st_boundary() const
{
  return path_st_boundary_;
}

const std::vector<std::string> &Obstacle::decider_tags() const
{
  return decider_tags_;
}

const std::vector<ObjectDecisionType> &Obstacle::decisions() const
{
  return decisions_;
}

const ObjectDecisionType &Obstacle::LongitudinalDecision() const
{
  return longitudinal_decision_;
}

const ObjectDecisionType &Obstacle::LateralDecision() const
{
  return lateral_decision_;
}

bool Obstacle::IsIgnore() const
{
  return IsLongitudinalIgnore() && IsLateralIgnore();
}

bool Obstacle::IsLongitudinalIgnore() const
{
  return longitudinal_decision_.IGNORE;
}

bool Obstacle::IsLateralIgnore() const
{
  return lateral_decision_.IGNORE;
}

const SLBoundary &Obstacle::PerceptionSLBoundary() const
{
  return sl_boundary_;
}

void Obstacle::set_path_st_boundary(const STBoundary &boundary)
{
  path_st_boundary_ = boundary;
  path_st_boundary_initialized_ = true;
}

void Obstacle::SetStBoundaryType(const STBoundary::BoundaryType type)
{
  path_st_boundary_.SetBoundaryType(type);
}

void Obstacle::EraseStBoundary() { path_st_boundary_ = STBoundary(); }

void Obstacle::SetReferenceLineStBoundary(const STBoundary &boundary)
{
  reference_line_st_boundary_ = boundary;
}

void Obstacle::SetReferenceLineStBoundaryType(
    const STBoundary::BoundaryType type)
{
  reference_line_st_boundary_.SetBoundaryType(type);
}

void Obstacle::EraseReferenceLineStBoundary()
{
  reference_line_st_boundary_ = STBoundary();
}

bool Obstacle::IsValidObstacle(
    const PerceptionObstacle &perception_obstacle)
{
  const double object_width = perception_obstacle.width;
  const double object_length = perception_obstacle.length;

  const double kMinObjectDimension = 1.0e-6;
  return !std::isnan(object_width) && !std::isnan(object_length) &&
         object_width > kMinObjectDimension &&
         object_length > kMinObjectDimension;
}

void Obstacle::SetLaneChangeBlocking(const bool is_distance_clear)
{
  is_lane_change_blocking_ = is_distance_clear;
}