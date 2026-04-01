#ifndef PNCPOINT
#define PNCPOINT


#include <string>
#include <vector>

struct SLPoint
{
  double s;
  double l;
};
struct SLBoundary
{
  double start_s;
  double end_s;
  double start_l;
  double end_l;
  SLPoint boundary_point;
};

struct FrenetFramePoint
{
  double s;
  double l;
  double dl;
  double ddl;
};

struct SpeedPoint
{
  double s;
  double t;
  // speed (m/s)
  double v;
  // acceleration (m/s^2)
  double a;
  // jerk (m/s^3)
  double da;
};

struct PathPoint
{
  // coordinates
  double x;
  double y;
  double z;

  // direction on the x-y plane
  double theta;
  // curvature on the x-y planning
  double kappa;
  // accumulated distance from beginning of the path
  double s;

  // derivative of kappa w.r.t s.
  double dkappa;
  // derivative of derivative of kappa w.r.t s.
  double ddkappa;
  // The lane ID where the path point is on
  std::string lane_id;

  // derivative of x and y w.r.t parametric parameter t in CosThetareferenceline
  double x_derivative;
  double y_derivative;
};

struct Path
{
  std::string name;
  PathPoint path_point;
};
struct GaussianInfo
{
  // Information of gaussian distribution
  double sigma_x;
  double sigma_y;
  double correlation;
  // Information of representative uncertainty area
  double area_probability;
  double ellipse_a;
  double ellipse_b;
  double theta_a;
};

struct TrajectoryPoint
{
  // path point
  PathPoint path_point;
  // linear velocity
  double v; // in [m/s]
            // linear acceleration
  double a;
  // relative time from beginning of the trajectory
  double relative_time;
  // longitudinal jerk
  double da;
  // The angle between vehicle front wheel and vehicle longitudinal axis
  double steer;

  // Gaussian probability information
  GaussianInfo gaussian_info;
};

struct Trajectory
{
  std::string name;
  TrajectoryPoint trajectory_point;
};

struct VehicleMotionPoint
{
  // trajectory point
  TrajectoryPoint trajectory_point;
  // The angle between vehicle front wheel and vehicle longitudinal axis
  double steer = 2;
};

struct VehicleMotion
{
  std::string name;
  VehicleMotionPoint vehicle_motion_point;
};
#endif