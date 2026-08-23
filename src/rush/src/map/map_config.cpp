#include "map/map_config.h"

#include <cmath>

namespace rush {

namespace {

bool finite_positive(float value) {
    return std::isfinite(value) && value > 0.f;
}

} // namespace

Map2dConfig::Map2dConfig(Point2f origin, float resolution_x, float resolution_y, int width,
                         int height)
    : origin_(origin), resolution_x_(resolution_x), resolution_y_(resolution_y), width_(width),
      height_(height) {}

bool Map2dConfig::is_valid() const {
    return std::isfinite(origin_.x) && std::isfinite(origin_.y) && finite_positive(resolution_x_) &&
           finite_positive(resolution_y_) && width_ > 0 && height_ > 0;
}

bool Map2dConfig::is_in_bound(const Point2i &grid) const {
    return grid.x >= 0 && grid.x < width_ && grid.y >= 0 && grid.y < height_;
}

bool Map2dConfig::is_in_bound(const Point2f &world) const {
    if (!is_valid() || !std::isfinite(world.x) || !std::isfinite(world.y)) {
        return false;
    }
    const auto min = world_min();
    const auto max = world_max();
    return world.x >= min.x && world.x < max.x && world.y >= min.y && world.y < max.y;
}

Point2f Map2dConfig::grid_to_world(const Point2i &grid) const {
    return Point2f{
        origin_.x + resolution_x_ * static_cast<float>(grid.x),
        origin_.y + resolution_y_ * static_cast<float>(grid.y),
    };
}

Point2i Map2dConfig::world_to_grid(const Point2f &world) const {
    return Point2i{
        static_cast<int>(std::floor((world.x - origin_.x) / resolution_x_)),
        static_cast<int>(std::floor((world.y - origin_.y) / resolution_y_)),
    };
}

const Point2f &Map2dConfig::origin() const {
    return origin_;
}

float Map2dConfig::resolution_x() const {
    return resolution_x_;
}

float Map2dConfig::resolution_y() const {
    return resolution_y_;
}

int Map2dConfig::width() const {
    return width_;
}

int Map2dConfig::height() const {
    return height_;
}

std::size_t Map2dConfig::cell_count() const {
    if (width_ <= 0 || height_ <= 0) {
        return 0;
    }
    return static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
}

Point2f Map2dConfig::world_min() const {
    return origin_;
}

Point2f Map2dConfig::world_max() const {
    return Point2f{
        origin_.x + resolution_x_ * static_cast<float>(width_),
        origin_.y + resolution_y_ * static_cast<float>(height_),
    };
}

Map3dConfig::Map3dConfig(Point3f origin, float resolution_x, float resolution_y, float resolution_z,
                         int width, int height, int depth)
    : origin_(origin), resolution_x_(resolution_x), resolution_y_(resolution_y),
      resolution_z_(resolution_z), width_(width), height_(height), depth_(depth) {}

bool Map3dConfig::is_valid() const {
    return std::isfinite(origin_.x) && std::isfinite(origin_.y) && std::isfinite(origin_.z) &&
           finite_positive(resolution_x_) && finite_positive(resolution_y_) &&
           finite_positive(resolution_z_) && width_ > 0 && height_ > 0 && depth_ > 0;
}

bool Map3dConfig::is_in_bound(const Point3i &grid) const {
    return grid.x >= 0 && grid.x < width_ && grid.y >= 0 && grid.y < height_ && grid.z >= 0 &&
           grid.z < depth_;
}

bool Map3dConfig::is_in_bound(const Point3f &world) const {
    if (!is_valid() || !std::isfinite(world.x) || !std::isfinite(world.y) ||
        !std::isfinite(world.z)) {
        return false;
    }
    const auto min = world_min();
    const auto max = world_max();
    return world.x >= min.x && world.x < max.x && world.y >= min.y && world.y < max.y &&
           world.z >= min.z && world.z < max.z;
}

Point3f Map3dConfig::grid_to_world(const Point3i &grid) const {
    return Point3f{
        origin_.x + resolution_x_ * static_cast<float>(grid.x),
        origin_.y + resolution_y_ * static_cast<float>(grid.y),
        origin_.z + resolution_z_ * static_cast<float>(grid.z),
    };
}

Point3i Map3dConfig::world_to_grid(const Point3f &world) const {
    return Point3i{
        static_cast<int>(std::floor((world.x - origin_.x) / resolution_x_)),
        static_cast<int>(std::floor((world.y - origin_.y) / resolution_y_)),
        static_cast<int>(std::floor((world.z - origin_.z) / resolution_z_)),
    };
}

const Point3f &Map3dConfig::origin() const {
    return origin_;
}

float Map3dConfig::resolution_x() const {
    return resolution_x_;
}

float Map3dConfig::resolution_y() const {
    return resolution_y_;
}

float Map3dConfig::resolution_z() const {
    return resolution_z_;
}

int Map3dConfig::width() const {
    return width_;
}

int Map3dConfig::height() const {
    return height_;
}

int Map3dConfig::depth() const {
    return depth_;
}

std::size_t Map3dConfig::voxel_count() const {
    if (width_ <= 0 || height_ <= 0 || depth_ <= 0) {
        return 0;
    }
    return static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_) *
           static_cast<std::size_t>(depth_);
}

Point3f Map3dConfig::world_min() const {
    return origin_;
}

Point3f Map3dConfig::world_max() const {
    return Point3f{
        origin_.x + resolution_x_ * static_cast<float>(width_),
        origin_.y + resolution_y_ * static_cast<float>(height_),
        origin_.z + resolution_z_ * static_cast<float>(depth_),
    };
}

} // namespace rush
