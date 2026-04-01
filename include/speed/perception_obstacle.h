#ifndef PERCEPTIONOBSTACLE
#define PERCEPTIONOBSTACLE

#include <string>
#include <vector>
#include <geometry.h>
#include "pnc_point.h"

struct BBox2D
{
  double xmin; // in pixels.
  double ymin; // in pixels.
  double xmax; // in pixels.
  double ymax; // in pixels.
};

struct LightStatus
{
  double brake_visible;
  double brake_switch_on;
  double left_turn_visible;
  double left_turn_switch_on;
  double right_turn_visible;
  double right_turn_switch_on;
};

struct V2XInformation
{
  enum V2XType
  {
    NONE,
    ZOMBIES_CAR,
    BLIND_ZONE,
  };
  V2XType v2x_type;
};

// struct Trajectory
// {
//   double probability; // probability of this trajectory
//   TrajectoryPoint trajectory_point;
// };

struct Debugstruct
{
  // can have multiple trajectories per obstacle
  Trajectory trajectory;
};
struct SensorMeasurement
{
  std::string sensor_id;
  int id;

  Point3D position;
  double theta;
  double length;
  double width;
  double height;

  Point3D velocity;

  enum Type
  {
    UNKNOWN,
    UNKNOWN_MOVABLE,
    UNKNOWN_UNMOVABLE,
    PEDESTRIAN, // Pedestrian, usually determined by moving behavior.
    BICYCLE,    // bike, motor bike
    VEHICLE,    // Passenger car or truck.
  };
  Type type; // obstacle type
  enum SubType
  {
    ST_UNKNOWN,
    ST_UNKNOWN_MOVABLE,
    ST_UNKNOWN_UNMOVABLE,
    ST_CAR,
    ST_VAN,
    ST_TRUCK,
    ST_BUS,
    ST_CYCLIST,
    ST_MOTORCYCLIST,
    ST_TRICYCLIST,
    ST_PEDESTRIAN,
    ST_TRAFFICCONE,
  };
  SubType sub_type; // obstacle sub_type
  double timestamp;
  BBox2D box; // only for camera measurements
};

struct PerceptionObstacle
{
  int id; // obstacle ID.

  // obstacle position in the world coordinate system.
  Point3D position;

  double theta;     // heading in the world coordinate system.
  Point3D velocity; // obstacle velocity.

  // Size of obstacle bounding box.
  double length; // obstacle length.
  double width;  // obstacle width.
  double height; // obstacle height.

  std::vector<Point3D> polygon_point; // obstacle corner points.
  int polygon_point_size;
  // duration of an obstacle since detection in s.
  double tracking_time;

  enum Type
  {
    UNKNOWN,
    UNKNOWN_MOVABLE,
    UNKNOWN_UNMOVABLE,
    PEDESTRIAN, // Pedestrian, usually determined by moving behavior.
    BICYCLE,    // bike, motor bike
    VEHICLE,    // Passenger car or truck.
  };
  Type type;        // obstacle type
  double timestamp; // GPS time in seconds.

  // Just for offline debugging, will not fill this field on board.
  // Format: [x0, y0, z0, x1, y1, z1...]
  double point_cloud;

  double confidence;
  enum ConfidenceType
  {
    CONFIDENCE_UNKNOWN,
    CONFIDENCE_CNN,
    CONFIDENCE_RADAR,
  };
  ConfidenceType confidence_type;
  // trajectory of object.
  Point3D drops;

  // The following fields are new added in Apollo 4.0
  Point3D acceleration; // obstacle acceleration

  // a stable obstacle point in the world coordinate system
  // position defined above is the obstacle bounding box ground center
  Point3D anchor_point;
  BBox2D bbox2d;

  enum SubType
  {
    ST_UNKNOWN,
    ST_UNKNOWN_MOVABLE,
    ST_UNKNOWN_UNMOVABLE,
    ST_CAR,
    ST_VAN,
    ST_TRUCK,
    ST_BUS,
    ST_CYCLIST,
    ST_MOTORCYCLIST,
    ST_TRICYCLIST,
    ST_PEDESTRIAN,
    ST_TRAFFICCONE,
  };
  SubType sub_type; // obstacle sub_type

  SensorMeasurement measurements; // sensor measurements

  // orthogonal distance between obstacle lowest point and ground plane
  double height_above_ground;

  // position covariance which is a row-majored 3x3 matrix
  double position_covariance;
  // velocity covariance which is a row-majored 3x3 matrix
  double velocity_covariance;
  // acceleration covariance which is a row-majored 3x3 matrix
  double acceleration_covariance;

  // lights of vehicles
  LightStatus light_status;

  // Debug struct
  Debugstruct msg;

  enum Source
  {
    HOST_VEHICLE,
    V2X,
  };

  Source source;
  V2XInformation v2x_info;
};
#endif