#pragma once

#include "nav_msgs/msg/odometry.hpp"
#include "robot_msgs/msg/local_plan.hpp"
#include "robot_msgs/msg/nav_event.hpp"
#include "robot_msgs/srv/nav_command.hpp"
#include "robot_msgs/srv/nav_executor_ctx.hpp"
#include "rush_extended/rush_extended.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/int8.hpp"

#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>

namespace rush {

class NavInterface : public rclcpp::Node {
public:
    NavInterface();
    ~NavInterface();

    RushExtended::Param load_params();

private:
    RushExtended rush_extended_;

    rclcpp::TimerBase::SharedPtr path_plan_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr replan_check_timer_;
    rclcpp::TimerBase::SharedPtr map_update_timer_;

    // interfaces
    using NavCommand = robot_msgs::srv::NavCommand;
    using LocalPlan = robot_msgs::msg::LocalPlan;
    using Odometry = nav_msgs::msg::Odometry;
    using NavExecutorCtx = robot_msgs::srv::NavExecutorCtx;
    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using NavEvent = robot_msgs::msg::NavEvent;

    rclcpp::Client<NavExecutorCtx>::SharedPtr nav_executor_ctx_cli_;
    rclcpp::Publisher<LocalPlan>::SharedPtr local_plan_pub_;
    rclcpp::Subscription<Odometry>::SharedPtr recv_odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr chassis_state_fdb_sub_;
    rclcpp::Subscription<Odometry>::SharedPtr localization_odom_sub_;
    rclcpp::Service<NavCommand>::SharedPtr nav_command_sub_;
    rclcpp::Subscription<PointCloud2>::SharedPtr pcl_sub_;
    rclcpp::Publisher<NavEvent>::SharedPtr nav_event_pub_;

    void nav_command_cbk(const NavCommand::Request::SharedPtr request,
                         NavCommand::Response::SharedPtr response);

    void pcl_cbk(const PointCloud2::SharedPtr msg);
};

} // namespace rush