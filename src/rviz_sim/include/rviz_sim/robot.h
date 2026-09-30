#pragma once

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <cstdint>

namespace gobot {

class Robot {
public:
    explicit Robot(float x, float y, std::int32_t initial_floor, float radius) noexcept;

    void move(float vx, float vy, float dt) noexcept;

    [[nodiscard]] sensor_msgs::msg::LaserScan
    detect(const nav_msgs::msg::OccupancyGrid::SharedPtr map) const noexcept;

    // Elevator behavior is intentionally left for a concrete integration to implement.
    void take_elevator(std::int32_t target_floor);

    [[nodiscard]] float velocity_x() const noexcept;
    [[nodiscard]] float velocity_y() const noexcept;
    [[nodiscard]] float radius() const noexcept;
    [[nodiscard]] float x() const noexcept;
    [[nodiscard]] float y() const noexcept;
    [[nodiscard]] std::int32_t current_floor() const noexcept;

private:
    float vx_{0.0F};
    float vy_{0.0F};
    float radius_{0.25F};
    float x_{0.0F};
    float y_{0.0F};
    std::int32_t current_floor_;
};

} // namespace gobot
