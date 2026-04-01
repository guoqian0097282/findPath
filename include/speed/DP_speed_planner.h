#ifndef DP_SPEED_PLANNER
#define DP_SPEED_PLANNER

#include <array>
#include <vector>
#include <iostream>
#include <limits>
#include <algorithm>
#include <unordered_map>
#include <string>
#include <memory>
#include <future> // std::async, std::future
#include <chrono> // std::chrono::milliseconds
#include <cmath>
#include <st_boundary.h>
#include <st_point.h>
#include <line_segment2d.h>
#include <math_utils.h>
double PtSegDistance(double query_x, double query_y, double start_x,
                     double start_y, double end_x, double end_y,
                     double length)
{
    const double x0 = query_x - start_x;
    const double y0 = query_y - start_y;
    const double dx = end_x - start_x;
    const double dy = end_y - start_y;
    const double proj = x0 * dx + y0 * dy;
    if (proj <= 0.0)
    {
        return hypot(x0, y0);
    }
    if (proj >= length * length)
    {
        return hypot(x0 - dx, y0 - dy);
    }
    return std::abs(x0 * dy - y0 * dx) / length;
}
class Box2dDP
{
public:
    Box2dDP(const std::vector<double> &center, const double heading, const double length,
          const double width)
    {
        // std::cout << "Box2dDP  " << center[0] << "  " << center[1] << std::endl;
        center_ = center;
        length_ = length;
        width_ = width;
        half_length_ = (length / 2.0);
        half_width_ = (width / 2.0);
        heading_ = (heading);
        cos_heading_ = (cos(heading));
        sin_heading_ = (sin(heading));
        InitCorners();
    }

    double center_x()
    {
        return center_[0];
    }

    double center_y()
    {
        return center_[1];
    }

    bool IsPointIn(const Vec2d &point) const
    {
        const double x0 = point.x() - center_[0];
        const double y0 = point.y() - center_[1];
        const double dx = std::abs(x0 * cos_heading_ + y0 * sin_heading_);
        const double dy = std::abs(-x0 * sin_heading_ + y0 * cos_heading_);
        return dx <= half_length_ + kMathEpsilon && dy <= half_width_ + kMathEpsilon;
    }
    double DistanceTo(const Vec2d &point) const
    {
        const double x0 = point.x() - center_[0];
        const double y0 = point.y() - center_[1];
        const double dx =
            std::abs(x0 * cos_heading_ + y0 * sin_heading_) - half_length_;
        const double dy =
            std::abs(x0 * sin_heading_ - y0 * cos_heading_) - half_width_;
        if (dx <= 0.0)
        {
            return std::max(0.0, dy);
        }
        if (dy <= 0.0)
        {
            return dx;
        }
        return hypot(dx, dy);
    }

    double DistanceTo(Box2dDP &point)
    {

        double x_diff = center_[0] - point.center_[0];
        double y_diff = center_[1] - point.center_[1];

        double distance = hypot(y_diff, x_diff);

        // distance = 0.5;
        return distance;
    }
    double DistanceTo(const LineSegment2d &line_segment) const
    {
        if (line_segment.length() <= kMathEpsilon)
        {
            return DistanceTo(line_segment.start());
        }
        const double ref_x1 = line_segment.start().x() - center_[0];
        const double ref_y1 = line_segment.start().y() - center_[1];
        double x1 = ref_x1 * cos_heading_ + ref_y1 * sin_heading_;
        double y1 = ref_x1 * sin_heading_ - ref_y1 * cos_heading_;
        double box_x = half_length_;
        double box_y = half_width_;
        int gx1 = (x1 >= box_x ? 1 : (x1 <= -box_x ? -1 : 0));
        int gy1 = (y1 >= box_y ? 1 : (y1 <= -box_y ? -1 : 0));
        if (gx1 == 0 && gy1 == 0)
        {
            return 0.0;
        }
        const double ref_x2 = line_segment.end().x() - center_[0];
        const double ref_y2 = line_segment.end().y() - center_[1];
        double x2 = ref_x2 * cos_heading_ + ref_y2 * sin_heading_;
        double y2 = ref_x2 * sin_heading_ - ref_y2 * cos_heading_;
        int gx2 = (x2 >= box_x ? 1 : (x2 <= -box_x ? -1 : 0));
        int gy2 = (y2 >= box_y ? 1 : (y2 <= -box_y ? -1 : 0));
        if (gx2 == 0 && gy2 == 0)
        {
            return 0.0;
        }
        if (gx1 < 0 || (gx1 == 0 && gx2 < 0))
        {
            x1 = -x1;
            gx1 = -gx1;
            x2 = -x2;
            gx2 = -gx2;
        }
        if (gy1 < 0 || (gy1 == 0 && gy2 < 0))
        {
            y1 = -y1;
            gy1 = -gy1;
            y2 = -y2;
            gy2 = -gy2;
        }
        if (gx1 < gy1 || (gx1 == gy1 && gx2 < gy2))
        {
            std::swap(x1, y1);
            std::swap(gx1, gy1);
            std::swap(x2, y2);
            std::swap(gx2, gy2);
            std::swap(box_x, box_y);
        }

        if (gx1 == 1 && gy1 == 1)
        {
            switch (gx2 * 3 + gy2)
            {
            case 4:
                return PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                     line_segment.length());
            case 3:
                return (x1 > x2) ? (x2 - box_x)
                                 : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                                 line_segment.length());
            case 2:
                return (x1 > x2) ? PtSegDistance(box_x, -box_y, x1, y1, x2, y2,
                                                 line_segment.length())
                                 : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                                 line_segment.length());
            case -1:
                return CrossProd({x1, y1}, {x2, y2}, {box_x, -box_y}) >= 0.0
                           ? 0.0
                           : PtSegDistance(box_x, -box_y, x1, y1, x2, y2,
                                           line_segment.length());
            case -4:
                return CrossProd({x1, y1}, {x2, y2}, {box_x, -box_y}) <= 0.0
                           ? PtSegDistance(box_x, -box_y, x1, y1, x2, y2,
                                           line_segment.length())
                           : (CrossProd({x1, y1}, {x2, y2}, {-box_x, box_y}) <= 0.0
                                  ? 0.0
                                  : PtSegDistance(-box_x, box_y, x1, y1, x2, y2,
                                                  line_segment.length()));
            }
        }
        else
        {
            switch (gx2 * 3 + gy2)
            {
            case 4:
                return (x1 < x2) ? (x1 - box_x)
                                 : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                                 line_segment.length());
            case 3:
                return std::min(x1, x2) - box_x;
            case 1:
            case -2:
                return CrossProd({x1, y1}, {x2, y2}, {box_x, box_y}) <= 0.0
                           ? 0.0
                           : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                           line_segment.length());
            case -3:
                return 0.0;
            }
        }
        return 0.0;
    }

    void InitCorners()
    {
        const double dx1 = cos_heading_ * half_length_;
        const double dy1 = sin_heading_ * half_length_;
        const double dx2 = sin_heading_ * half_width_;
        const double dy2 = -cos_heading_ * half_width_;
        corners_.clear();

        std::vector<double> corner_point_1{center_[0] + dx1 + dx2, center_[1] + dy1 + dy2};
        std::vector<double> corner_point_2{center_[0] + dx1 - dx2, center_[1] + dy1 - dy2};
        std::vector<double> corner_point_3{center_[0] - dx1 - dx2, center_[1] - dy1 - dy2};
        std::vector<double> corner_point_4{center_[0] - dx1 + dx2, center_[1] - dy1 + dy2};

        corners_.push_back(corner_point_1);
        corners_.push_back(corner_point_2);
        corners_.push_back(corner_point_3);
        corners_.push_back(corner_point_4);

        for (auto &corner : corners_)
        {
            max_x_ = std::fmax(corner[0], max_x_);
            min_x_ = std::fmin(corner[0], min_x_);
            max_y_ = std::fmax(corner[1], max_y_);
            min_y_ = std::fmin(corner[1], min_y_);
        }
    }
    std::vector<double> center_;
    double length_ = 0.0;
    double width_ = 0.0;
    double half_length_ = 0.0;
    double half_width_ = 0.0;
    double heading_ = 0.0;
    double cos_heading_ = 1.0;
    double sin_heading_ = 0.0;

    double max_x_ = std::numeric_limits<double>::lowest();
    double min_x_ = std::numeric_limits<double>::max();
    double max_y_ = std::numeric_limits<double>::lowest();
    double min_y_ = std::numeric_limits<double>::max();

    std::vector<std::vector<double>> corners_;

    bool HasOverlap(const Box2dDP &box) const
    {
        if (box.max_x_ < min_x_ || box.min_x_ > max_x_ || box.max_y_ < min_y_ ||
            box.min_y_ > max_y_)
        {
            return false;
        }
    }
    bool HasOverlap(LineSegment2d &line_segment) const
    {
        if (line_segment.length() <= kMathEpsilon)
        {
            return IsPointIn(line_segment.start());
        }
        if (std::fmax(line_segment.start().x(), line_segment.end().x()) < min_x_ ||
            std::fmin(line_segment.start().x(), line_segment.end().x()) > max_x_ ||
            std::fmax(line_segment.start().y(), line_segment.end().y()) < min_y_ ||
            std::fmin(line_segment.start().y(), line_segment.end().y()) > max_y_)
        {
            return false;
        }
        return DistanceTo(line_segment) <= kMathEpsilon;
    }
};

// 数据部分
struct Waypoint
{
    Waypoint(const double _x, const double _y, const double _theta, const double _s)
        : x(_x), y(_y), theta(_theta), s(_s)
    {
    }
    double x;
    double y;
    double theta;
    double s;
};

struct DiscretizedPathDP
{
    explicit DiscretizedPathDP(const std::vector<Waypoint> &_waypoints) : points(_waypoints)
    {
    }
    DiscretizedPathDP() = default;
    std::vector<Waypoint> points;
};
struct TrajectoryPointOB
{
    TrajectoryPointOB(const Waypoint &_waypoint, const double _v, const double _relative_time)
        : waypoint(_waypoint), v(_v), relative_time(_relative_time)
    {
    }
    Waypoint waypoint;
    double v;
    double relative_time;
};
struct DiscretizedTrajectoryOB
{
    explicit DiscretizedTrajectoryOB(const std::vector<TrajectoryPointOB> &_traj_points)
        : traj_points(_traj_points)
    {
    }
    DiscretizedTrajectoryOB() = default;
    std::vector<TrajectoryPointOB> traj_points;
};

// 障碍物
struct ObstacleDP
{
    ObstacleDP(const std::string &_id, const Box2dDP &_bounding_box, const bool _is_static,
               const DiscretizedTrajectoryOB &_trajectory)
        : id(_id), bounding_box(_bounding_box), is_static(_is_static), trajectory(_trajectory)
    {
    }
    ObstacleDP(const std::string &_id, const Box2dDP &_bounding_box, const bool _is_static)
        : id(_id), bounding_box(_bounding_box), is_static(_is_static)
    {
    }
    std::string id;
    Box2dDP bounding_box;
    bool is_static;
    DiscretizedTrajectoryOB trajectory;
};

// defined for cyber task
struct StGraphMessage
{
    StGraphMessage(const uint32_t c_, const int32_t r_) : c(c_), r(r_) {}
    uint32_t c;
    uint32_t r;
};

// S-T Graph
class StGraph final
{
public:
    StGraph(const DiscretizedPathDP &path, const std::vector<ObstacleDP> &obstacles);

    bool GetAllObstacleSTBoundary(std::vector<STBoundary> *const st_boundaries) const;

    bool GetObstacleSTBoundary(const std::string &obs_id, STBoundary *const st_boundary) const;

private:
    void CalculateAllObstacleSTBoundary(const std::vector<ObstacleDP> &obstacles);

    bool CalculateObstacleSTBoundary(const ObstacleDP &obs, STBoundary *const st_boundary) const;

    bool CalculateStaticObstacleSTBoundary(const std::string &obs_id, const Box2dDP &obs_box,
                                           STBoundary *const st_boundary) const;

    bool CalculateDynamicObstacleSTBoundary(const ObstacleDP &obs, STBoundary *const st_boundary) const;

    std::vector<Waypoint> GetWaypointsWithinDistance(const double x, const double y,
                                                     const double radius) const;

    bool GetBlockedSRange(const Box2dDP &obs_box, double *const lower_s, double *const upper_s) const;

private:
    std::unordered_map<std::string, STBoundary> obs_st_boundary_;
    DiscretizedPathDP discretized_path_;

    const double veh_heading_ = M_PI / 4.0;
    const double veh_length_ = 2.0;
    const double veh_width_ = 1.0;
    const double time_range_ = 4.0;
};

StGraph::StGraph(const DiscretizedPathDP &path, const std::vector<ObstacleDP> &obstacles)
    : discretized_path_(path)
{
    // CHECK_GE(discretized_path_.points.size(), 2);

    obs_st_boundary_.clear();
    CalculateAllObstacleSTBoundary(obstacles);
}

bool StGraph::GetAllObstacleSTBoundary(std::vector<STBoundary> *const st_boundaries) const
{
    // CHECK_NOTNULL(st_boundaries);

    for (auto iter = obs_st_boundary_.begin(); iter != obs_st_boundary_.end(); ++iter)
    {
        st_boundaries->emplace_back(iter->second);
    }
    return true;
}

bool StGraph::GetObstacleSTBoundary(const std::string &obs_id,
                                    STBoundary *const st_boundary) const
{
    // CHECK_NOTNULL(st_boundary);

    const auto &iter = obs_st_boundary_.find(obs_id);
    if (iter == obs_st_boundary_.end())
    {
        return false;
    }
    *st_boundary = iter->second;
    return true;
}

void StGraph::CalculateAllObstacleSTBoundary(const std::vector<ObstacleDP> &obstacles)
{
    for (const auto &obstacle : obstacles)
    {
        const auto &iter = obs_st_boundary_.find(obstacle.id);
        // CHECK(iter == obs_st_boundary_.end());
        STBoundary st_boundary;
        if (!CalculateObstacleSTBoundary(obstacle, &st_boundary))
        {
            // AWARN << "Failed to get obstacle: " << obstacle.id << " st_boundary. ";
            continue;
        }
        obs_st_boundary_.insert(std::make_pair(obstacle.id, st_boundary));
    }
}

bool StGraph::CalculateObstacleSTBoundary(const ObstacleDP &obstacle,
                                          STBoundary *const st_boundary) const
{
    // CHECK_NOTNULL(st_boundary);

    if (obstacle.is_static)
    {
        return CalculateStaticObstacleSTBoundary(obstacle.id, obstacle.bounding_box, st_boundary);
    }
    else
    {
        return CalculateDynamicObstacleSTBoundary(obstacle, st_boundary);
    }
}

bool StGraph::CalculateStaticObstacleSTBoundary(const std::string &obs_id, const Box2dDP &obs_box,
                                                STBoundary *const st_boundary) const
{
    // CHECK_NOTNULL(st_boundary);

    double lower_s = 0.0;
    double upper_s = 0.0;
    if (!GetBlockedSRange(obs_box, &lower_s, &upper_s))
    {
        return false;
    }
    std::vector<STPoint> lower_points{STPoint(lower_s, 0.0), STPoint(lower_s, time_range_)};
    std::vector<STPoint> upper_points{STPoint(upper_s, 0.0), STPoint(upper_s, time_range_)};
    st_boundary->Init(obs_id, lower_points, upper_points);
    return true;
}

bool StGraph::GetBlockedSRange(const Box2dDP &obs_box, double *const lower_s,
                               double *const upper_s) const
{
    // CHECK_NOTNULL(lower_s);
    // CHECK_NOTNULL(upper_s);

    const double radius = 5.0;
    double radiAus = 2.0;
    const auto &points = GetWaypointsWithinDistance(obs_box.center_[0], obs_box.center_[1], radiAus);
    if (points.empty())
    {
        std::cout << "Obstacle is not on the path. ";
        return false;
    }

    bool is_updated = false;

    for (auto iter = points.begin(); iter != points.end(); ++iter)
    {
        Box2dDP veh_box({iter->x, iter->y}, veh_heading_, veh_length_, veh_width_);
        if (veh_box.HasOverlap(obs_box))
        {
            iter = iter == points.begin() ? iter : iter - 1;
            *lower_s = iter->s;
            is_updated = true;
            break;
        }
    }

    if (!is_updated)

    {
        std::cout << "There is no collision with obstacle. ";
        return false;
    }

    for (auto iter = points.rbegin(); iter != points.rend(); ++iter)
    {
        Box2dDP veh_box({iter->x, iter->y}, veh_heading_, veh_length_, veh_width_);
        if (veh_box.HasOverlap(obs_box))
        {
            iter = iter == points.rbegin() ? iter : iter - 1;
            *upper_s = iter->s;
            break;
        }
    }
    // CHECK_GE(*upper_s, *lower_s) << "upper_s: " << *upper_s << ", lower_s: " << *lower_s;
    return true;
}

bool StGraph::CalculateDynamicObstacleSTBoundary(const ObstacleDP &obstacle,
                                                 STBoundary *const st_boundary) const
{
    // CHECK_NOTNULL(st_boundary);
    // CHECK_GE(obstacle.trajectory.traj_points.size(), 2);

    bool has_st_boundary = false;
    std::vector<STPoint> lower_points;
    std::vector<STPoint> upper_points;
    for (const auto &p : obstacle.trajectory.traj_points)
    {
        Box2dDP obs_box({p.waypoint.x, p.waypoint.y}, p.waypoint.theta, obstacle.bounding_box.length_,
                      obstacle.bounding_box.width_);
        double lower_s = 0.0;
        double upper_s = 0.0;
        if (GetBlockedSRange(obs_box, &lower_s, &upper_s))
        {
            lower_points.emplace_back(lower_s, p.relative_time);
            upper_points.emplace_back(upper_s, p.relative_time);
            has_st_boundary = true;
        }
    }
    // lower_points.emplace_back(30.0, 4.0);
    // lower_points.emplace_back(30.0, 6.0);
    // upper_points.emplace_back(45.0, 4.0);
    // upper_points.emplace_back(45.0, 6.0);
    st_boundary->Init(obstacle.id, lower_points, upper_points);
    return has_st_boundary;
}

std::vector<Waypoint> StGraph::GetWaypointsWithinDistance(const double x, const double y,
                                                          const double radius) const
{
    std::vector<Waypoint> nearest_points;
    for (const auto &p : discretized_path_.points)
    {
        if (std::hypot(p.x - x, p.y - y) < radius)
        {
            nearest_points.emplace_back(p);
        }
    }
    return nearest_points;
}
#endif