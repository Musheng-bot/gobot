from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_dir = Path(get_package_share_directory("robot_gazebo"))
    description_dir = Path(get_package_share_directory("robot_description"))
    world = gazebo_dir / "worlds" / "simple_room.sdf"
    model = description_dir / "urdf" / "robot.urdf.xacro"

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(get_package_share_directory("gazebo_ros")) / "launch" / "gazebo.launch.py")
            ),
            launch_arguments={
                "world": str(world),
                "gui": "true",
                "server_required": "true",
            }.items(),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": Command(["xacro ", str(model), " use_sim:=true"])}],
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", "diffbot",
                "-topic", "robot_description",
                "-x", "-2.5", "-y", "-2.0", "-z", "0.15",
                "-unpause",
            ],
            output="screen",
        ),
    ])
