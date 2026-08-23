#pragma once

#include "base.h"
#include "map/occupancy_2d.h"

#include <optional>
#include <string>

namespace rush {

class AStar {
public:
    struct PlanResult {
        std::optional<Path2f> path;
        std::string detail;
    };

    PlanResult plan(const Pose &robot_pose, const Pose &goal_pose, const Occupancy2d &occ_map);
};

} // namespace rush