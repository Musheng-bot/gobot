#include "rviz_sim/simulator.h"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

namespace gobot {

constexpr float VELOCITY_NOISE_RATIO = 0.02F;
constexpr auto TICK_PERIOD = std::chrono::milliseconds(50);

Simulator::Simulator(const std::string &node_name)
    : Node(node_name), robot_(static_cast<float>(declare_parameter("initial_x", 0.0)),
                              static_cast<float>(declare_parameter("initial_y", 0.0)),
                              declare_parameter<std::int32_t>("initial_floor", 1),
                              static_cast<float>(declare_parameter("robot_radius", 0.25))) {
    command_timeout_ = declare_parameter("command_timeout", command_timeout_);

    control_plan_sub_ = create_subscription<robot_msgs::msg::ControlPlan>(
        "/gobot/control_plan", 1,
        std::bind(&Simulator::control_plan_callback, this, std::placeholders::_1));

    const auto map_qos = rclcpp::QoS(1).transient_local().reliable();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/gobot/map", map_qos);
    lidar_scan_pub_ =
        create_publisher<sensor_msgs::msg::LaserScan>("/gobot/scan", rclcpp::QoS(1).reliable());
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/gobot/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    const auto scene_config = declare_parameter<std::string>("scene_config", "");
    if (scene_config.empty()) {
        RCLCPP_WARN(get_logger(), "Parameter scene_config is empty; no floor maps were loaded");
    } else if (!layered_map_.load_config(scene_config)) {
        RCLCPP_ERROR(get_logger(), "Failed to load layered map config: %s", scene_config.c_str());
    } else {
        publish_current_map(get_clock()->now());
    }

    last_command_time_ = get_clock()->now();
    timer_ = create_wall_timer(TICK_PERIOD, std::bind(&Simulator::tick, this));
}

void Simulator::control_plan_callback(const robot_msgs::msg::ControlPlan::SharedPtr msg) {
    command_vx_ = msg->vx;
    command_vy_ = msg->vy;
    last_command_time_ = get_clock()->now();
}

const nav_msgs::msg::OccupancyGrid::SharedPtr Simulator::current_map() const noexcept {
    return layered_map_.get_map(robot_.current_floor());
}

void Simulator::publish_current_map(const rclcpp::Time &stamp) {
    const auto map = current_map();
    if (map == nullptr) {
        RCLCPP_ERROR_ONCE(get_logger(), "No map configured for floor %d", robot_.current_floor());
        map_published_ = false;
        return;
    }

    auto published_map = *map;
    published_map.header.stamp = stamp;
    published_map.header.frame_id = "map";
    map_pub_->publish(published_map);
    published_floor_ = robot_.current_floor();
    map_published_ = true;
    RCLCPP_INFO(get_logger(), "Published floor %d on /gobot/map", published_floor_);
}

bool Simulator::world_to_map(const double world_x, const double world_y, std::size_t &map_x,
                             std::size_t &map_y) const {
    const auto map = current_map();
    if (map == nullptr) {
        return false;
    }

    const auto &origin = map->info.origin;
    const double yaw = 2.0 * std::atan2(origin.orientation.z, origin.orientation.w);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const double dx = world_x - origin.position.x;
    const double dy = world_y - origin.position.y;
    const double local_x = cos_yaw * dx + sin_yaw * dy;
    const double local_y = -sin_yaw * dx + cos_yaw * dy;

    if (local_x < 0.0 || local_y < 0.0) {
        return false;
    }

    map_x = static_cast<std::size_t>(local_x / map->info.resolution);
    map_y = static_cast<std::size_t>(local_y / map->info.resolution);
    return map_x < map->info.width && map_y < map->info.height;
}

bool Simulator::occupied(const double world_x, const double world_y) const {
    const auto map = current_map();
    std::size_t map_x = 0;
    std::size_t map_y = 0;
    if (map == nullptr || !world_to_map(world_x, world_y, map_x, map_y)) {
        return true;
    }

    const auto index = map_y * map->info.width + map_x;
    const auto occupancy = map->data[index];
    return LayeredMap::is_occupied(occupancy);
}

bool Simulator::collides(const double world_x, const double world_y) const {
    const auto map = current_map();
    if (map == nullptr) {
        return true;
    }

    const double step = std::max(0.01, static_cast<double>(map->info.resolution) * 0.5);
    const double radius = robot_.radius();
    const double radius_squared = radius * radius;

    for (double dx = -radius; dx <= radius; dx += step) {
        for (double dy = -radius; dy <= radius; dy += step) {
            if (dx * dx + dy * dy <= radius_squared && occupied(world_x + dx, world_y + dy)) {
                return true;
            }
        }
    }

    return occupied(world_x, world_y);
}

void Simulator::publish_state(const rclcpp::Time &stamp) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = robot_.x();
    odom.pose.pose.position.y = robot_.y();
    odom.pose.pose.orientation.w = 1.0;
    odom.twist.twist.linear.x = robot_.velocity_x();
    odom.twist.twist.linear.y = robot_.velocity_y();
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = robot_.x();
    transform.transform.translation.y = robot_.y();
    transform.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(transform);
}

void Simulator::tick() {
    const auto now = this->get_clock()->now();
    const auto dt = std::chrono::duration<float>(TICK_PERIOD).count();

    if (!map_published_ || published_floor_ != robot_.current_floor()) {
        publish_current_map(now);
    }

    if ((now - last_command_time_).seconds() > command_timeout_) {
        command_vx_ = 0.0F;
        command_vy_ = 0.0F;
    }

    if (current_map() != nullptr) {
        const auto step = static_cast<float>(dt);
        const float speed = std::hypot(command_vx_, command_vy_);
        float actual_vx = command_vx_;
        float actual_vy = command_vy_;
        if (speed > 0.0F) {
            std::normal_distribution<float> velocity_noise(0.0F, speed * VELOCITY_NOISE_RATIO);
            actual_vx += velocity_noise(random_engine_);
            actual_vy += velocity_noise(random_engine_);
        }

        const double next_x = robot_.x() + actual_vx * step;
        const double next_y = robot_.y() + actual_vy * step;
        if (!collides(next_x, next_y)) {
            robot_.move(actual_vx, actual_vy, step);
        } else {
            robot_.move(0.0F, 0.0F, step);
            command_vx_ = 0.0F;
            command_vy_ = 0.0F;
        }
    } else {
        RCLCPP_WARN_ONCE(get_logger(), "Waiting for a map for floor %d", robot_.current_floor());
        robot_.move(0.0F, 0.0F, static_cast<float>(dt));
        command_vx_ = 0.0F;
        command_vy_ = 0.0F;
    }

    auto scan = robot_.detect(current_map());
    scan.header.stamp = now;
    publish_state(now);
    lidar_scan_pub_->publish(scan);
}

} // namespace gobot
