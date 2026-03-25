//
// Created by guoqian on 23-10-1.
//
#include <gflags/gflags.h>
#include <cmath>
#include "config/planning_flags.hpp"

///// Car params.
/////
DEFINE_double(car_width, 2.0, "");

DEFINE_double(car_length, 4.9, "");

DEFINE_double(safety_margin, 0.5, "mandatory safety margin");

DEFINE_double(wheel_base, 2.8, "wheel base");

DEFINE_double(rear_length, 1.0, "rear axle to rear edge");

DEFINE_double(front_length, 3.9, "rear axle to front edge");

DEFINE_double(max_steering_angle, 22.0 * M_PI / 180.0, "");
/////

///// Smoothing related.
/////
DEFINE_string(smoothing_method, "TENSION2", "rReference smoothing method");

DEFINE_bool(enable_searching, true, "search before optimization");

DEFINE_double(search_lateral_range, 3.0, "max offset when searching");

DEFINE_double(search_longitudial_spacing, 1.5, "longitudinal spacing when searching");

DEFINE_double(search_lateral_spacing, 0.6, "lateral spacing when searching");

// TODO: change names!
DEFINE_double(cartesian_curvature_weight, 1, "");

DEFINE_double(cartesian_curvature_rate_weight, 50, "");

DEFINE_double(cartesian_deviation_weight, 1.0, "");

DEFINE_double(tension_2_deviation_weight, 1.005, "");

DEFINE_double(tension_2_curvature_weight, 1, "");

DEFINE_double(tension_2_curvature_rate_weight, 50, "");

DEFINE_double(search_obstacle_cost, 0.4, "searching cost");

DEFINE_double(search_deviation_cost, 0.8, "offset from the original ref cost");
/////

///// Optimization related

DEFINE_double(expected_safety_margin, 0.6, "soft constraint on the distance to obstacles"); //0.6

// TODO: make this work.
DEFINE_bool(constraint_end_heading, true, "add constraints on end heading");

///// Others.
/////
DEFINE_double(output_spacing, 0.5, "output interval");

DEFINE_double(epsilon, 1e-6, "use this when comparing double");

DEFINE_bool(enable_dynamic_segmentation, true, "dense segmentation when the curvature is large.");

DEFINE_bool(rough_constraints_far_away, false, "Use rough collision constraints after some distance, controlled by precise_planning_length");

DEFINE_double(precise_planning_length, 30.0, "More strict collision constraint.");
/////