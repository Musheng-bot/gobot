#include "rush_extended/rush_extended.h"

#include "base.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>

namespace rush {

void RushExtended::tick_plan(const TopographyPath &topography_path) {
    const auto REF_PATH = geometric_topo_map_.translate_to_real_path(topography_path);
    const auto global_path =
        astar_planner_.plan(robot_pose_, plan_cmd_pack_.goal_pose, global_map_);
}

} // namespace rush