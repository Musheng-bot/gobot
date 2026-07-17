from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_dir = Path(get_package_share_directory("robot_description"))
    model = description_dir / "urdf" / "robot.urdf.xacro"

    return LaunchDescription([
        DeclareLaunchArgument("use_lidar", default_value="true"),
        DeclareLaunchArgument("use_camera", default_value="true"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "robot_description": Command([
                    "xacro ", str(model),
                    " use_sim:=false",
                    " use_lidar:=", LaunchConfiguration("use_lidar"),
                    " use_camera:=", LaunchConfiguration("use_camera"),
                ]),
            }],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", str(description_dir / "rviz" / "robot.rviz")],
        ),
    ])
