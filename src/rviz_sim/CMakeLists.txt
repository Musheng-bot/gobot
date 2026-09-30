cmake_minimum_required(VERSION 3.8)
project(rviz_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_library(rviz_sim_pkg STATIC
  src/layered_map.cpp
  src/robot.cpp
  src/simulator.cpp
)
target_compile_features(rviz_sim_pkg PUBLIC cxx_std_17)
target_include_directories(rviz_sim_pkg PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rviz_sim_pkg
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  robot_msgs
  tf2_ros
  nav2_map_server
  yaml_cpp_vendor
)
target_link_libraries(rviz_sim_pkg yaml-cpp)

add_executable(simulator_node src/simulator_node.cpp)
target_link_libraries(simulator_node rviz_sim_pkg)
ament_target_dependencies(simulator_node rclcpp)

install(TARGETS rviz_sim_pkg
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS simulator_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY scene DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  robot_msgs
  tf2_ros
  nav2_map_server
  yaml_cpp_vendor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
