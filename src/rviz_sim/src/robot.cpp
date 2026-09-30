#include "rviz_sim/robot.h"
#include "rviz_sim/layered_map.h"

#include <algorithm>
#include <cmath>

namespace gobot {

constexpr std::size_t LIDAR_SAMPLE_COUNT = 360;
constexpr float LIDAR_RANGE_MIN = 0.05F;
constexpr float LIDAR_RANGE_MAX = 10.0F;
constexpr double PI = 3.14159265358979323846;

bool world_to_map(const nav_msgs::msg::OccupancyGrid &map, const double world_x,
                  const double world_y, std::size_t &map_x, std::size_t &map_y) {
    const auto &origin = map.info.origin;
    const double yaw = 2.0 * std::atan2(origin.orientation.z, origin.orientation.w);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const double dx = world_x - origin.position.x;
    const double dy = world_y - origin.position.y;
    const double local_x = cos_yaw * dx + sin_yaw * dy;
    const double local_y = -sin_yaw * dx + cos_yaw * dy;

    if (map.info.resolution <= 0.0F || local_x < 0.0 || local_y < 0.0) {
        return false;
    }

    map_x = static_cast<std::size_t>(local_x / map.info.resolution);
    map_y = static_cast<std::size_t>(local_y / map.info.resolution);
    return map_x < map.info.width && map_y < map.info.height;
}

bool occupied(const nav_msgs::msg::OccupancyGrid &map, const double world_x,
              const double world_y) {
    std::size_t map_x = 0;
    std::size_t map_y = 0;
    if (!world_to_map(map, world_x, world_y, map_x, map_y)) {
        return true;
    }

    return LayeredMap::is_occupied(map.data[map_y * map.info.width + map_x]);
}

Robot::Robot(const float x, const float y, const std::int32_t initial_floor,
             const float radius) noexcept
    : radius_(radius), x_(x), y_(y), current_floor_(initial_floor) {}

void Robot::move(const float vx, const float vy, const float dt) noexcept {
    vx_ = vx;
    vy_ = vy;
    x_ += vx * dt;
    y_ += vy * dt;
}

sensor_msgs::msg::LaserScan
Robot::detect(const nav_msgs::msg::OccupancyGrid::SharedPtr map) const noexcept {
    sensor_msgs::msg::LaserScan lidar_scan;
    lidar_scan.header.frame_id = "base_link";
    lidar_scan.angle_min = static_cast<float>(-PI);
    lidar_scan.angle_increment = static_cast<float>(2.0 * PI / LIDAR_SAMPLE_COUNT);
    lidar_scan.angle_max = lidar_scan.angle_min + static_cast<float>((LIDAR_SAMPLE_COUNT - 1) *
                                                                     lidar_scan.angle_increment);
    lidar_scan.scan_time = 0.05F;
    lidar_scan.time_increment = 0.0F;
    lidar_scan.range_min = LIDAR_RANGE_MIN;
    lidar_scan.range_max = LIDAR_RANGE_MAX;
    lidar_scan.ranges.assign(LIDAR_SAMPLE_COUNT, LIDAR_RANGE_MAX);

    if (map == nullptr) {
        return lidar_scan;
    }

    const double step = std::max(0.01, static_cast<double>(map->info.resolution) * 0.5);
    for (std::size_t i = 0; i < LIDAR_SAMPLE_COUNT; ++i) {
        const double angle = lidar_scan.angle_min + i * lidar_scan.angle_increment;
        for (double range = LIDAR_RANGE_MIN; range <= LIDAR_RANGE_MAX; range += step) {
            const double point_x = x_ + range * std::cos(angle);
            const double point_y = y_ + range * std::sin(angle);
            if (occupied(*map, point_x, point_y)) {
                lidar_scan.ranges[i] = static_cast<float>(range);
                break;
            }
        }
    }

    return lidar_scan;
}

float Robot::velocity_x() const noexcept {
    return vx_;
}

float Robot::velocity_y() const noexcept {
    return vy_;
}

float Robot::radius() const noexcept {
    return radius_;
}

float Robot::x() const noexcept {
    return x_;
}

float Robot::y() const noexcept {
    return y_;
}

std::int32_t Robot::current_floor() const noexcept {
    return current_floor_;
}

} // namespace gobot
