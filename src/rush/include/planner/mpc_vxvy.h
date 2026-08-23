#pragma once

#include "base.h"

#include <utility>

namespace rush {

class MpcVxVy {
public:
    std::pair<float, float> plan_speed(const Pose &pose, const Path2f &path);
};

} // namespace rush