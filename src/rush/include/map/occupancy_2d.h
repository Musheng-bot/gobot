#pragma once

#include "base.h"
#include "map/map_config.h"

namespace rush {

class Occupancy2d {
public:
    bool is_in_bound(const Point2f &p) const;

    bool is_occupied(const Point2f &p) const;

    float occupation(const Point2f &p) const;

private:
    Map2dConfig config_;
};

} // namespace rush