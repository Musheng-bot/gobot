#pragma once

#include <vector>

namespace rush {

struct Point2i {
    int x;
    int y;
};

struct Point2f {
    float x;
    float y;
};

struct Point3i {
    int x;
    int y;
    int z;
};

struct Point3f {
    float x;
    float y;
    float z;
};

using Path2i = std::vector<Point2i>;
using Path2f = std::vector<Point2f>;

struct Pose {
    Point2f position;
    float yaw; // [-pi, pi]
};

enum class ControlMode { VX_VY = 0, V_OMEGA = 1 };

enum class ChassisState {
    NORMAL = 0,
};

enum class Topography {};

using TopographyPath = std::vector<Topography>;
using ChassisStatePath = std::vector<ChassisState>;

Point2f project_to_path(const Point2f &point, const Path2f &path);

} // namespace rush
