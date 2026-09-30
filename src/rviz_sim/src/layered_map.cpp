#include "rviz_sim/layered_map.h"

#include "nav2_map_server/map_io.hpp"
#include "yaml-cpp/yaml.h"

#include <filesystem>
#include <utility>

namespace gobot {

const nav_msgs::msg::OccupancyGrid::SharedPtr
LayeredMap::get_map(const std::int32_t floor) const noexcept {
    const auto map = maps_.find(floor);
    return map == maps_.end() ? nav_msgs::msg::OccupancyGrid::SharedPtr{} : map->second;
}

bool LayeredMap::is_occupied(const std::int8_t occupancy) noexcept {
    return occupancy < 0 || occupancy >= 50;
}

void LayeredMap::update_map(const std::int32_t floor, nav_msgs::msg::OccupancyGrid map) {
    maps_.insert_or_assign(
        floor, std::make_shared<nav_msgs::msg::OccupancyGrid>(std::move(map)));
}

bool LayeredMap::load_config(const std::string &config_path) {
    try {
        const auto config = YAML::LoadFile(config_path);
        const auto floors = config["floors"];
        if (!floors || !floors.IsSequence()) {
            return false;
        }

        const auto config_directory = std::filesystem::absolute(config_path).parent_path();
        std::unordered_map<std::int32_t, nav_msgs::msg::OccupancyGrid::SharedPtr> loaded_maps;

        for (const auto &floor_config : floors) {
            const auto floor = floor_config["floor"].as<std::int32_t>();
            const auto map_path =
                (config_directory / floor_config["map"].as<std::string>()).lexically_normal();

            nav_msgs::msg::OccupancyGrid map;
            if (nav2_map_server::loadMapFromYaml(map_path.string(), map) !=
                nav2_map_server::LOAD_MAP_SUCCESS) {
                return false;
            }
            map.header.frame_id = "map";
            loaded_maps.insert_or_assign(
                floor, std::make_shared<nav_msgs::msg::OccupancyGrid>(std::move(map)));
        }

        if (loaded_maps.empty()) {
            return false;
        }

        maps_ = std::move(loaded_maps);
        return true;
    } catch (const std::exception &) {
        return false;
    }
}

} // namespace gobot
