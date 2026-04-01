#ifndef DECISION
#define DECISION

#include <string>
#include <vector>

#include "geometry.h";

struct TargetLane
{
  // lane id
  std::string id;
  double start_s;     // in meters
  double end_s;       // in meters
  double speed_limit; // in m/s
};

struct ObjectIgnore
{
};

enum StopReasonCode
{
  STOP_REASON_HEAD_VEHICLE,
  STOP_REASON_DESTINATION,
  STOP_REASON_PEDESTRIAN,
  STOP_REASON_OBSTACLE,
  STOP_REASON_PREPARKING,
  STOP_REASON_SIGNAL, // only for red signal
  STOP_REASON_STOP_SIGN,
  STOP_REASON_YIELD_SIGN,
  STOP_REASON_CLEAR_ZONE,
  STOP_REASON_CROSSWALK,
  STOP_REASON_CREEPER,
  STOP_REASON_REFERENCE_END, // end of the reference_line
  STOP_REASON_YELLOW_SIGNAL, // yellow signal
  STOP_REASON_PULL_OVER,     // pull over
  STOP_REASON_SIDEPASS_SAFETY,
  STOP_REASON_PRE_OPEN_SPACE_STOP,
  STOP_REASON_LANE_CHANGE_URGENCY,
  STOP_REASON_EMERGENCY,
};

struct ObjectStop
{
  StopReasonCode reason_code;
  double distance_s; // in meters
                     // When stopped, the front center of vehicle should be at this point.
  PointENU stop_point;
  // When stopped, the heading of the vehicle should be stop_heading.
  double stop_heading;
  std::string wait_for_obstacle;
};

// dodge the obstacle in lateral direction when driving
struct ObjectNudge
{
  enum Type
  {
    LEFT_NUDGE,          // drive from the left side to nudge a static obstacle
    RIGHT_NUDGE,         // drive from the right side to nudge a static obstacle
    DYNAMIC_LEFT_NUDGE,  // drive from the left side to nudge a dynamic obstacle
    DYNAMIC_RIGHT_NUDGE, // drive from the right side to nudge a dynamic obstacle
  };
  Type type;
  // minimum lateral distance in meters. positive if type = LEFT_NUDGE
  // negative if type = RIGHT_NUDGE
  double distance_l;
};

struct ObjectYield
{
  double distance_s; // minimum longitudinal distance in meters
  PointENU fence_point;
  double fence_heading;
  double time_buffer; // minimum time buffer required after the
                      // obstacle reaches the intersect point.
};

struct ObjectFollow
{
  double distance_s; // minimum longitudinal distance in meters
  PointENU fence_point;
  double fence_heading;
};

struct ObjectOvertake
{
  double distance_s; // minimum longitudinal distance in meters
  PointENU fence_point;
  double fence_heading;
  double time_buffer; // minimum time buffer required before the
                      // obstacle reaches the intersect point.
};

struct ObjectSidePass
{
  enum Type
  {
    LEFT,
    RIGHT,
  };
  Type type;
};

// unified object decision while estop
struct ObjectAvoid
{
};

struct ObjectStatic
{
};

struct ObjectDynamic
{
};

struct ObjectMotionType
{
  enum Motion_tag
  {
    STATIC,
    DYNAMIC,
  };
  Motion_tag motion_tag;
};

struct ObjectDecisionType
{
  enum ObjectTagCase
  {
    IGNORE,
    STOP,
    FOLLOW,
    YIELD,
    OVERTAKE,
    NUDGE,
    AVOIDE,
    SIDE_PASS,
  };
  ObjectTagCase object_tag;
};

struct ObjectDecision
{
  std::string id;
  int perception_id;
  ObjectDecisionType object_decision;
};
struct ObjectStatus
{
  ObjectMotionType motion_type;
  ObjectDecisionType decision_type;
};
struct ObjectDecisions
{
  ObjectDecision decision;
};
enum ChangeLaneType
{
  FORWARD,
  LEFT,
  RIGHT,
};
struct MainStop
{
  StopReasonCode reason_code;
  std::string reason;
  // When stopped, the front center of vehicle should be at this point.
  PointENU stop_point;
  // When stopped, the heading of the vehicle should be stop_heading.
  double stop_heading = 4;
  ChangeLaneType change_lane_type;
};

struct EmergencyStopHardBrake
{
};

struct EmergencyStopCruiseToStop
{
};

struct MainEmergencyStop
{
  // Unexpected event happened, human driver is required to take over
  enum ReasonCode
  {
    ESTOP_REASON_INTERNAL_ERR,
    ESTOP_REASON_COLLISION,
    ESTOP_REASON_ST_FIND_PATH,
    ESTOP_REASON_ST_MAKE_DECISION,
    ESTOP_REASON_SENSOR_ERROR,
  };
  ReasonCode reason_code;
  std::string reason;
  enum Task
  {
    EHARD_BRAKE,    // hard brake
    CRUISE_TO_STOP, // cruise to stop
  };
  Task task;
};

struct MainCruise
{
  // cruise current lane
  ChangeLaneType change_lane_type;
};

// This struct is deprecated
struct MainChangeLane
{
  enum Type
  {
    LEFT,
    RIGHT,
  };
  Type type;
  TargetLane default_lane;
  MainStop default_lane_stop;
  MainStop target_lane_stop;
};

struct MainMissionComplete
{
  // arrived at routing destination
  // When stopped, the front center of vehicle should be at this point.
  PointENU stop_point;
  // When stopped, the heading of the vehicle should be stop_heading.
  double stop_heading;
};

struct MainNotReady
{
  // decision system is not ready.
  // e.g. wait for routing data.
  std::string reason;
};

struct MainParking
{
  enum ParkingStatus
  {
    // TODO(QiL): implement and expand to more enums
    IN_PARKING,
  };
  ParkingStatus status;
};

struct MainDecision
{
  enum Task
  {
    CRUISE,
    STOP,
    ESTOP,
    CHANGE_LANE,
    MissionComplete,
    NotReady,
    Parking,
  };
  Task task;
  TargetLane target_lane;
};
#endif