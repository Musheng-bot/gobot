from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("rviz_sim")
    config_file = PathJoinSubstitution([package_share, "config", "sim.yaml"])
    rviz_config = PathJoinSubstitution([package_share, "rviz", "sim.rviz"])

    return LaunchDescription(
        [
            Node(
                package="rviz_sim",
                executable="simulator_node",
                name="simulator",
                namespace="gobot",
                parameters=[config_file],
                cwd=package_share,
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
