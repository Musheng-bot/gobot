#pragma once

#include "base.h"

#include <memory>

namespace rush {

class Map2d {
public:
    using SharedPtr = std::shared_ptr<Map2d>;

    Map2d(const Point2f &origin, float resoltion_x, float resolution_y, int width, int height);
    virtual ~Map2d();

    Point2f grid_to_world(const Point2i &p) const;

    Point2i world_to_grid(const Point2f &p) const;

    const Point2f &origin() const;

    float resolution_x() const;

    virtual bool is_occupied(const Point2f &p) const = 0;

    virtual float occupation(const Point2f &p) const = 0;

private:
    Point2f origin_;
    float resolution_x_;
    float resolution_y_;
    int width_;
    int height_;
};

} // namespace rush