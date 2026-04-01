
#pragma once

#include "speed_data.h"
#include "st_graph_data.h"

class SpeedOptimizer {
 public:
  explicit SpeedOptimizer();

  virtual ~SpeedOptimizer() = default;
  // bool Execute(Frame* frame,
  //                        ReferenceLineInfo* reference_line_info) override;

//  protected:
//   virtual bool Process(const PathData& path_data,
//                                  const TrajectoryPoint& init_point,
//                                  SpeedData* const speed_data) = 0;

//   void RecordDebugInfo(const SpeedData& speed_data);
};
