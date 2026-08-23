#pragma once

#include "base.h"

namespace rush {

class MpcVOmega {
public:
    std::pair<float, float> plan_speed(const Pose &pose, const Path2f &path);
};
} // namespace rush