#pragma once

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/control_plan.hpp"
#include "rviz_sim/layered_map.h"
#include "rviz_sim/robot.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <random>
#include <string>

namespace gobot {

class Simulator : public rclcpp::Node {
public:
    explicit Simulator(const std::string &node_name);
    void tick();

private:
    void control_plan_callback(const robot_msgs::msg::ControlPlan::SharedPtr msg);
    void publish_current_map(const rclcpp::Time &stamp);

    [[nodiscard]] const nav_msgs::msg::OccupancyGrid::SharedPtr
    current_map() const noexcept;

    [[nodiscard]] bool world_to_map(double world_x, double world_y, std::size_t &map_x,
                                    std::size_t &map_y) const;
    [[nodiscard]] bool occupied(double world_x, double world_y) const;
    [[nodiscard]] bool collides(double world_x, double world_y) const;
    void publish_state(const rclcpp::Time &stamp);

    Robot robot_;
    LayeredMap layered_map_;
    std::int32_t published_floor_{0};
    bool map_published_{false};

    double command_timeout_{0.5};
    float command_vx_{0.0F};
    float command_vy_{0.0F};

    rclcpp::Subscription<robot_msgs::msg::ControlPlan>::SharedPtr control_plan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_scan_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Time last_command_time_;
    std::mt19937 random_engine_{std::random_device{}()};
};

} // namespace gobot
