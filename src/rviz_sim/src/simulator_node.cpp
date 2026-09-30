#include "rclcpp/rclcpp.hpp"
#include "rviz_sim/simulator.h"

#include <memory>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gobot::Simulator>("simulator"));
    rclcpp::shutdown();
    return 0;
}
