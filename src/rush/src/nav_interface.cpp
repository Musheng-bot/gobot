#include "nav_interface.h"

#include "base.h"
#include "rush_extended/rush_extended.h"

#include <Eigen/Eigen>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>

namespace rush {
NavInterface::NavInterface() : rclcpp::Node("rush") {
    nav_command_sub_ = this->create_subscription<NavCommand>(
        "/rush_extended/input/nav_command", 10,
        [this](const NavCommand::SharedPtr msg) { this->nav_command_cbk(msg); });
}

void NavInterface::nav_command_cbk(const NavCommand::SharedPtr msg) {
    const float W = msg->goal.orientation.w;
    const float X = msg->goal.orientation.x;
    const float Y = msg->goal.orientation.y;
    const float Z = msg->goal.orientation.z;
    const float YAW = std::atan2(2 * (W * Z + X * Y), 1 - 2 * (Y * Y + Z * Z));

    const Pose GOAL_POSE{
        Point2f{static_cast<float>(msg->goal.position.x), static_cast<float>(msg->goal.position.y)},
        YAW};
    if (msg->command == NavCommand::START) {
        rush_extended_.set_goal(GOAL_POSE);
        rush_extended_.set_nav_mode(RushExtended::NavMode::NORMAL);
    } else if (msg->command == NavCommand::CHASE_START ||
               msg->command == NavCommand::CHASE_UPDATE) {
        rush_extended_.set_goal(GOAL_POSE);
        rush_extended_.set_nav_mode(RushExtended::NavMode::CHASE);
    } else if (msg->command == NavCommand::CANCEL || msg->command == NavCommand::CHASE_STOP) {
        rush_extended_.set_nav_mode(RushExtended::NavMode::IDLE);
    } else {
        RCLCPP_WARN(this->get_logger(), "Unknown command value in \'NavCommand\': %d",
                    msg->command);
    }
}

} // namespace rush