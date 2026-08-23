#pragma once

#include "base.h"

#include <cstddef>

namespace rush {

class Map2dConfig {
public:
    Map2dConfig() = default;
    Map2dConfig(Point2f origin, float resolution_x, float resolution_y, int width, int height);

    bool is_valid() const;
    bool is_in_bound(const Point2i &grid) const;
    bool is_in_bound(const Point2f &world) const;

    Point2f grid_to_world(const Point2i &grid) const;
    Point2i world_to_grid(const Point2f &world) const;

    const Point2f &origin() const;
    float resolution_x() const;
    float resolution_y() const;
    int width() const;
    int height() const;
    std::size_t cell_count() const;

    Point2f world_min() const;
    Point2f world_max() const;

private:
    Point2f origin_{0.f, 0.f};
    float resolution_x_ = 0.f;
    float resolution_y_ = 0.f;
    int width_ = 0;
    int height_ = 0;
};

class Map3dConfig {
public:
    Map3dConfig() = default;
    Map3dConfig(Point3f origin, float resolution_x, float resolution_y, float resolution_z,
                int width, int height, int depth);

    bool is_valid() const;
    bool is_in_bound(const Point3i &grid) const;
    bool is_in_bound(const Point3f &world) const;

    Point3f grid_to_world(const Point3i &grid) const;
    Point3i world_to_grid(const Point3f &world) const;

    const Point3f &origin() const;
    float resolution_x() const;
    float resolution_y() const;
    float resolution_z() const;
    int width() const;
    int height() const;
    int depth() const;
    std::size_t voxel_count() const;

    Point3f world_min() const;
    Point3f world_max() const;

private:
    Point3f origin_{0.f, 0.f, 0.f};
    float resolution_x_ = 0.f;
    float resolution_y_ = 0.f;
    float resolution_z_ = 0.f;
    int width_ = 0;
    int height_ = 0;
    int depth_ = 0;
};

} // namespace rush
