#pragma once

#include "robot_msgs/msg/nav_command.hpp"
#include "rush_extended/rush_extended.h"

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

namespace rush {

class NavInterface : public rclcpp::Node {
public:
    NavInterface();

private:
    RushExtended rush_extended_;

    RushExtended::Param load_params();

    using NavCommand = robot_msgs::msg::NavCommand;

    rclcpp::Subscription<NavCommand>::SharedPtr nav_command_sub_;

    void nav_command_cbk(NavCommand::SharedPtr msg);
};

} // namespace rush