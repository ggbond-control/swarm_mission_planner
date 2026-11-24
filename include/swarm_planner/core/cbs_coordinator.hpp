#pragma once

#include <swarm_planner/core/grid_path_planner.hpp>

namespace swarm_planner
{
    class CbsCoordinator
    {
    public:
        explicit CbsCoordinator(const GridMap &map);

        std::vector<std::vector<TimedState>> coordinate(const std::vector<Robot> &robots, const std::vector<RobotRoute> &routes) const;

    private:
        const GridMap &_map;
    };
} // namespace swarm_planner
