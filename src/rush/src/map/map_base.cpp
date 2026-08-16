#include "map/map_base.h"

#include "base.h"

#include <cmath>

namespace rush {
Map2d::Map2d(const Point2f &origin, float resoltion_x, float resolution_y, int width, int height)
    : origin_(origin), resolution_x_(resoltion_x), resolution_y_(resolution_y), width_(width),
      height_(height) {}

Map2d::~Map2d() = default;

Point2f Map2d::grid_to_world(const Point2i &p) const {
    return Point2f{resolution_x_ * p.x + origin_.x, resolution_y_ * p.y + origin_.y};
}

Point2i Map2d::world_to_grid(const Point2f &p) const {
    return Point2i{static_cast<int>(std::floor((p.x - origin_.x) / resolution_x_)),
                   static_cast<int>(std::floor((p.y - origin_.y) / resolution_y_))};
}

const Point2f &Map2d::origin() const {
    return origin_;
}

} // namespace rush