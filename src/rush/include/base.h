#pragma once

#include <vector>

namespace gobot {

struct Point2i {
    int x;
    int y;
};

struct Point2f {
    float x;
    float y;
};

using Path2i = std::vector<Point2i>;
using Path2f = std::vector<Point2f>;

} // namespace gobot
