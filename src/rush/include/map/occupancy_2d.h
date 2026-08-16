#pragma once

#include "base.h"
#include "map/map_base.h"

namespace rush {

class Occupancy2d : public Map2d {
public:
    bool is_occupied(const Point2f &p) const override;

    float occupation(const Point2f &p) const override;
};

} // namespace rush