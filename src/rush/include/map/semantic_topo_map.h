#pragma once

#include "base.h"
#include "map/map_config.h"

#include <vector>

namespace rush {

class SemanticTopoMap {
public:
    std::vector<TopographyPath> plan_topography_path(const Pose &start, const Pose &goal);
};

} // namespace rush