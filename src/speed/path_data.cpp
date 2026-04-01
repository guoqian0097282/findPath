#include "path_data.h"
#include <cmath>
#include <algorithm>
#include <iostream>
// #include "absl/strings/str_cat.h"
// #include "absl/strings/str_join.h"
// #include "log.h"
// #include "planning_gflags.h"


bool PathData::SetPathPointDecisionGuide(
    std::vector<std::tuple<double, PathPointType, double>>
        path_point_decision_guide) {
  // if (reference_line_ == nullptr) {
  //   std::cout << "Should NOT set path_point_decision_guide when reference line is "
  //             "nullptr. ";
  //   return false;
  // }
  // if (frenet_path_.empty() || discretized_path_.empty()) {
  //   std::cout << "Should NOT set path_point_decision_guide when frenet_path or "
  //             "world frame trajectory is empty. ";
  //   return false;
  // }
  path_point_decision_guide_ = std::move(path_point_decision_guide);
  return true;
}

const DiscretizedPath &PathData::discretized_path() const {
  return discretized_path_;
}

const std::vector<std::tuple<double, PathData::PathPointType, double>>
    &PathData::path_point_decision_guide() const {
  return path_point_decision_guide_;
}

// bool PathData::Empty() const {
//   return discretized_path_.empty() && frenet_path_.empty();
// }

// void PathData::SetReferenceLine(const ReferenceLine *reference_line) {
//   Clear();
//   reference_line_ = reference_line;
// }

PathPoint PathData::GetPathPointWithPathS(const double s) const {
  return discretized_path_.Evaluate(s);
}


void PathData::Clear() {
  discretized_path_.clear();
  frenet_path_.clear();
  path_point_decision_guide_.clear();
  path_reference_.clear();
  // reference_line_ = nullptr;
}


void PathData::set_path_label(const std::string &label) { path_label_ = label; }

const std::string &PathData::path_label() const { return path_label_; }

const std::vector<PathPoint> &PathData::path_reference() const {
  return path_reference_;
}

void PathData::set_path_reference(
    const std::vector<PathPoint> &path_reference) {
  path_reference_ = std::move(path_reference);
}