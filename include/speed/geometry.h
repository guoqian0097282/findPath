#ifndef GEWOMETRY
#define GEWOMETRY

#include <string>
#include <vector>

struct PointENU
{
  double x = 1; // [default = nan]; // East from the origin, in meters.
  double y = 2; // [default = nan]; // North from the origin, in meters.
  double z = 3; // [default = 0.0]; // Up from the WGS-84 ellipsoid, in
                // meters.
};

// A point in the global reference frame. Similar to PointENU, PointLLH allows
// omitting the height field for representing a 2D location.
struct PointLLH
{
  // Longitude in degrees, ranging from -180 to 180.
  double lon = 1; // [default = nan];
  // Latitude in degrees, ranging from -90 to 90.
  double lat = 2; // [default = nan];
  // WGS-84 ellipsoid height in meters.
  double height = 3; // [default = 0.0];
};

// A general 2D point. Its meaning and units depend on context, and must be
// explained in comments.
struct Point2D
{
  double x = 1; // [default = nan];
  double y = 2; // [default = nan];
};

// A general 3D point. Its meaning and units depend on context, and must be
// explained in comments.
struct Point3D
{
  double x = 1; //[default = nan];
  double y = 2; // [default = nan];
  double z = 3; //[default = nan];
};

// A unit quaternion that represents a spatial rotation. See the link below for
// details.
//   https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
// The scalar part qw can be omitted. In this case, qw should be calculated by
//   qw = sqrt(1 - qx * qx - qy * qy - qz * qz).
struct Quaternion
{
  double qx = 1; // [default = nan];
  double qy = 2; //[default = nan];
  double qz = 3; // [default = nan];
  double qw = 4; // [default = nan];
};

// A general polygon, points are counter clockwise
struct Polygon
{
  Point3D point;
};
#endif