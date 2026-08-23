#include "nav_interface.h"

#include "base.h"
#include "rush_extended/rush_extended.h"

#include <Eigen/Eigen>
#include <chrono>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/detail/int8__struct.hpp>

namespace rush {
NavInterface::NavInterface() : rclcpp::Node("rush"), rush_extended_() {
    using namespace std::chrono_literals;

    nav_executor_ctx_cli_ = this->create_client<NavExecutorCtx>("/rush/nav_executor_ctx");
    local_plan_pub_ = this->create_publisher<LocalPlan>("/rush/plan/local_plan", 10);
    nav_event_pub_ = this->create_publisher<NavEvent>("/rush/status/nav_event", 10);

    recv_odom_sub_ = this->create_subscription<Odometry>("/protocol/recv_odom", 10,
                                                         [this](const Odometry::SharedPtr msg) {});

    localization_odom_sub_ = this->create_subscription<Odometry>(
        "/Odometry/imu_interpolation", 10, [this](const Odometry::SharedPtr msg) {});

    nav_command_sub_ = this->create_service<NavCommand>(
        "/rush/input/nav_command", [this](const NavCommand::Request::SharedPtr request,
                                          NavCommand::Response::SharedPtr response) {
            this->nav_command_cbk(request, response);
        });

    chassis_state_fdb_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "/rush/input/chassis_state_fdb", 10, [this](const std_msgs::msg::Int8::SharedPtr msg) {});

    pcl_sub_ = this->create_subscription<PointCloud2>(
        "/cloud_registered", 10, [this](const PointCloud2::SharedPtr msg) { this->pcl_cbk(msg); });

    map_update_timer_ =
        this->create_wall_timer(50ms, [this]() { rush_extended_.tick_map_update(); });
    path_plan_timer_ = this->create_wall_timer(100ms, [this]() {
        // rush_extended_.tick_plan();
    });
    control_timer_ = this->create_wall_timer(20ms, [this]() { rush_extended_.tick_control(); });
    replan_check_timer_ =
        this->create_wall_timer(100ms, [this]() { rush_extended_.tick_replan_check(); });
}

void NavInterface::pcl_cbk(const PointCloud2::SharedPtr msg) {}

// void NavInterface::nav_command_cbk(const NavCommand::SharedPtr msg) {
//     const float W = msg->goal.orientation.w;
//     const float X = msg->goal.orientation.x;
//     const float Y = msg->goal.orientation.y;
//     const float Z = msg->goal.orientation.z;
//     const float YAW = std::atan2(2 * (W * Z + X * Y), 1 - 2 * (Y * Y + Z * Z));

//     const Pose GOAL_POSE{
//         Point2f{static_cast<float>(msg->goal.position.x),
//         static_cast<float>(msg->goal.position.y)}, YAW};
//     if (msg->command == NavCommand::START) {
//         rush_extended_.set_goal(GOAL_POSE);
//         rush_extended_.set_nav_mode(RushExtended::NavMode::NORMAL);
//     } else if (msg->command == NavCommand::CHASE_START ||
//                msg->command == NavCommand::CHASE_UPDATE) {
//         rush_extended_.set_goal(GOAL_POSE);
//         rush_extended_.set_nav_mode(RushExtended::NavMode::CHASE);
//     } else if (msg->command == NavCommand::CANCEL || msg->command == NavCommand::CHASE_STOP)
//     {
//         rush_extended_.set_nav_mode(RushExtended::NavMode::IDLE);
//     } else {
//         RCLCPP_WARN(this->get_logger(), "Unknown command value in \'NavCommand\': %d",
//                     msg->command);
//     }
// }

} // namespace rush
