from __future__ import annotations

import os
import random
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

CIRCUITS = ("monza", "silverstone", "spa", "suzuka", "monaco")


def _base_launch(circuit: str) -> LaunchDescription:
    if circuit not in CIRCUITS:
        raise RuntimeError(f"Unknown circuit '{circuit}'. Choose from: {', '.join(CIRCUITS)}")

    pkg_share = Path(get_package_share_directory("urrc_track_gazebo"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    world = pkg_share / "worlds" / f"{circuit}.sdf"

    # model://urrc_track_gazebo/... resolves from the parent share directory.
    resource_root = str(pkg_share.parent)
    previous = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = resource_root if not previous else resource_root + os.pathsep + previous

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_share / "launch" / "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r -v 3 {world}"}.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        LogInfo(msg=f"[urrc_2] circuit={circuit} | vehicle model intentionally not spawned"),
        gazebo,
    ])


def fixed_circuit_launch(circuit: str) -> LaunchDescription:
    return _base_launch(circuit)


def random_circuit_launch() -> LaunchDescription:
    return _base_launch(random.choice(CIRCUITS))
