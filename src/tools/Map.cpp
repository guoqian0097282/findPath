//
// Created by guoqian on 23-10-1.
//
#include "tools/Map.hpp"

namespace PathOptimizationNS {

Map::Map(const grid_map::GridMap &grid_map) :
    maps(grid_map) {
    if (!grid_map.exists("distance")) {
        std::cout << "grid map must contain 'distance' layer";
    }
}

double Map::getObstacleDistance(const Eigen::Vector2d &pos) const {
    if (maps.isInside(pos)) {
        return this->maps.atPosition("distance", pos, grid_map::InterpolationMethods::INTER_LINEAR);
    } else {
        return 0.0;
    }
}

bool Map::isInside(const Eigen::Vector2d &pos) const {
    return maps.isInside(pos);
}
}