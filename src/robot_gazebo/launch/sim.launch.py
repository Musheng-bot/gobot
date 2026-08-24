from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = Path(get_package_share_directory("robot_gazebo"))
    desc_share = Path(get_package_share_directory("robot_description"))
    gazebo_launch = Path(get_package_share_directory("gazebo_ros")) / "launch" / "gazebo.launch.py"
    world = sim_share / "worlds" / "objectnav_room.sdf"
    robot = desc_share / "urdf" / "robot.urdf.xacro"
    nav_launch = Path(get_package_share_directory("robot_navigation")) / "launch" / "navigation.launch.py"

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false", description="Start Gazebo Classic GUI"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo_launch)),
            launch_arguments={"world": str(world), "gui": LaunchConfiguration("gui"), "verbose": "false"}.items(),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": Command(["xacro ", str(robot), " use_sim:=true"]), "use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-entity", "gobot", "-topic", "robot_description", "-x", "-2.5", "-y", "-1.8", "-z", "0.12"],
            output="screen",
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav_launch))),
        Node(package="robot_perception", executable="target_detector", parameters=[{"use_sim_time": True}], output="screen"),
        Node(package="robot_objectnav", executable="target_navigator", parameters=[{"use_sim_time": True}], output="screen"),
    ])
