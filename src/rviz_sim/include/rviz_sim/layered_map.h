#pragma once

#include "nav_msgs/msg/occupancy_grid.hpp"

#include <cstdint>
#include <string>
#include <unordered_map>

namespace gobot {

class LayeredMap {
public:
    [[nodiscard]] const nav_msgs::msg::OccupancyGrid::SharedPtr
    get_map(std::int32_t floor) const noexcept;

    [[nodiscard]] static bool is_occupied(std::int8_t occupancy) noexcept;
    void update_map(std::int32_t floor, nav_msgs::msg::OccupancyGrid map);
    [[nodiscard]] bool load_config(const std::string &config_path);

private:
    std::unordered_map<std::int32_t, nav_msgs::msg::OccupancyGrid::SharedPtr> maps_;
};

} // namespace gobot
