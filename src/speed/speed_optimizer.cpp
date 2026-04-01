#include "speed_optimizer.h"

// #include "planning_gflags.h"
#include "speed_limit.h"


// SpeedOptimizer::SpeedOptimizer(){}

// bool SpeedOptimizer::Execute(Frame* frame,
//                                ReferenceLineInfo* reference_line_info) {
//   Task::Execute(frame, reference_line_info);

//   auto ret =
//       Process(reference_line_info->path_data(), frame->PlanningStartPoint(),
//               reference_line_info->mutable_speed_data());

//   RecordDebugInfo(reference_line_info->speed_data());
//   return ret;
// }

// void SpeedOptimizer::RecordDebugInfo(const SpeedData& speed_data) {
//   auto* debug = reference_line_info_->mutable_debug();
//   auto ptr_speed_plan = debug->mutable_planning_data()->add_speed_plan();
//   ptr_speed_plan->set_name(Name());
//   ptr_speed_plan->mutable_speed_point()->CopyFrom(
//       {speed_data.begin(), speed_data.end()});
// }