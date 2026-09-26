"""Gazebo-only world; no robot, ROS topics or network settings are created."""
import sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    def start(context):
        package = Path(get_package_share_directory("urrc_track_gazebo"))
        cmd = [sys.executable, str(package / "scripts/run_world.py"), "monza",
               "--view", LaunchConfiguration("view").perform(context)]
        if LaunchConfiguration("server").perform(context).lower() in ("true", "1"):
            cmd.append("--server")
        return [ExecuteProcess(cmd=cmd, output="screen")]
    return LaunchDescription([
        DeclareLaunchArgument("view", default_value="overview", choices=["overview", "grid"]),
        DeclareLaunchArgument("server", default_value="false"),
        OpaqueFunction(function=start),
    ])
