//
// Created by guoqian on 23-10-1.
//

#ifndef PATH_OPTIMIZER__PATHOPTIMIZER_HPP_p
#define PATH_OPTIMIZER__PATHOPTIMIZER_HPP_

#include <string>
#include <vector>
#include <memory>
#include "grid_map_core/grid_map_core.hpp"
//#include <tuple>
//#include <glog/logging.h>

namespace PathOptimizationNS {

class ReferencePath;
class State;
class SlState;
class Map;
class CollisionChecker;
class VehicleState;

class PathOptimizer {
public:
    PathOptimizer() = delete;
    PathOptimizer(const State &start_state,
                  const State &end_state,
                  const grid_map::GridMap &map);
    PathOptimizer(const PathOptimizer &optimizer) = delete;
    PathOptimizer &operator=(const PathOptimizer &optimizer) = delete;

    // Call this to get the optimized path.
    bool solve(const std::vector<State> &reference_points, std::vector<SlState> *final_path);
    void printObstacleDistance(std::vector<SlState> path);
    void printObstacleDistance2(std::vector<State> path);
    double calDisObs(double x,double y);
    // Only for visualization purpose.
//    std::vector<std::tuple<State, double, double>> display_abnormal_bounds() const;
    const ReferencePath &getReferencePath() const;

private:
    // Core function.
    bool optimizePath(std::vector<SlState> *final_path);

    // Divide smoothed path into segments.
    bool processReferencePath();

    // Calculate init error between the vehicle and the ref.
    void processInitState();

    //
    void setReferencePathLength();

    std::shared_ptr<Map> grid_map_;
    std::shared_ptr<CollisionChecker> collision_checker_;
    std::shared_ptr<ReferencePath> reference_path_;
    std::shared_ptr<VehicleState> vehicle_state_;
};
}

#endif //PATH_OPTIMIZER__PATHOPTIMIZER_HPP_
