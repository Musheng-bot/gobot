from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    nav_share = Path(get_package_share_directory("robot_navigation"))
    slam_launch = Path(get_package_share_directory("slam_toolbox")) / "launch" / "online_async_launch.py"
    nav_launch = Path(get_package_share_directory("nav2_bringup")) / "launch" / "navigation_launch.py"
    params = nav_share / "config" / "nav2_params.yaml"
    common = {"use_sim_time": "True", "params_file": str(params)}
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(slam_launch)),
            launch_arguments={"use_sim_time": "True"}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav_launch)),
            launch_arguments=common.items(),
        ),
    ])
