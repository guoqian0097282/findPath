#ifndef PREDICTOBSTACLE
#define PREDICTOBSTACLE

#include <string>
#include <vector>
#include "perception_obstacle.h";

// estimated obstacle intent
struct ObstacleIntent
{
  enum Type
  {
    UNKNOWN,
    STOP,
    STATIONARY,
    MOVING,
    CHANGE_LANE,
    LOW_ACCELERATION,
    HIGH_ACCELERATION,
    LOW_DECELERATION,
    HIGH_DECELERATION,
  };
  Type type;
};

// self driving car intent
struct Intent
{
  enum Type
  {
    UNKNOWN,
    STOP,
    CRUISE,
    CHANGE_LANE,
  };
  Type type;
};
struct ObstaclePriority
{
  enum Priority
  {
    CAUTION,
    NORMAL,
    IGNORE,
  };
  Priority priority;
};

struct TrajectoryObstacle
{
  double probability; // probability of this trajectory
  std::vector<TrajectoryPoint> trajectory_point;
};
struct LaneFeature {
   std::string lane_id;
   int lane_turn_type ;
   double lane_s ;
   double lane_l;
   double angle_diff ;
   double dist_to_left_boundary ;
   double dist_to_right_boundary ;
   double lane_heading ;
  //  apollo.hdmap.Lane.LaneType lane_type = 9;
};

struct Lane {
  // Features of all possible current lanes.
   LaneFeature current_lane_feature ;

  // Features of the most possible current lane.
   LaneFeature lane_feature ;

  // Features of all nearby lanes.
   LaneFeature nearby_lane_feature;

  // Lane graph
  //  LaneGraph lane_graph ;
  //  LaneGraph lane_graph_ordered ;

  // For modeling
   double label_update_time_delta;
};
// next id = 37
struct Feature
{
  // Obstacle ID
  int id;

  // Obstacle features
  Point3D polygon_point;
  Point3D position;
  Point3D front_position;
  Point3D velocity;
  Point3D raw_velocity; // from perception
  Point3D acceleration;
  double velocity_heading;
  double speed;
  double acc;
  double theta;
  double length;
  double width;
  double height;
  double tracking_time;
  double timestamp;

  // Obstacle type-specific features
  Lane lane ;
  // JunctionFeature junction_feature = 26;

  // Obstacle tracked features
  Point3D t_position;
  Point3D t_velocity;
  double t_velocity_heading;
  double t_speed;
  Point3D t_acceleration;
  double t_acc;

  bool is_still;
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
  double label_update_time_delta;

  ObstaclePriority priority;

  bool is_near_junction;

  // Obstacle ground-truth labels:
  // PredictionTrajectoryPoint future_trajectory_points;

  // Obstacle short-term predicted trajectory points
  TrajectoryPoint
      short_term_predicted_trajectory_points;

  // Obstacle predicted trajectories
  TrajectoryObstacle predicted_trajectory;

  // ADC trajectory at the same frame
  TrajectoryPoint adc_trajectory_point;

  // Surrounding lanes
  std::string surrounding_lane_id;
  std::string within_lane_id;
};

struct PredictionObstacle
{
  PerceptionObstacle perception_obstacle;
  double timestamp; // GPS time in seconds
                    // the length of the time for this prediction (e.g. 10s)
  double predicted_period;
  // can have multiple trajectories per obstacle
  std::vector<TrajectoryObstacle> trajectory;

  // estimated obstacle intent
  ObstacleIntent intent;

  ObstaclePriority priority;

  bool is_static;

  // Feature history latest -> earliest sequence
  Feature feature;
};

struct PredictionObstacles
{
  // timestamp is included in header
  // Header header = 1;

  // make prediction for multiple obstacles
  std::vector<PredictionObstacle> prediction_obstacle;

  // perception error code
  // ErrorCode perception_error_code;

  // start timestamp
  double start_timestamp;

  // end timestamp
  double end_timestamp;

  // self driving car intent
  Intent intent;

  // Scenario
  // Scenario scenario;
};
#endif