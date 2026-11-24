#pragma once

#include <swarm_planner/offline/offline_map_planner.hpp>

namespace swarm_planner::offline
{
    std::vector<std::vector<TimedMapState>> coordinate_multi_map_routes(
        const MultiMapPathPlanner &path_planner,
        const std::vector<MappedRobot> &robots,
        const std::vector<MappedRobotRoute> &routes,
        CoordinationStats *stats = nullptr);
} // namespace swarm_planner::offline
