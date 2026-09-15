from glob import glob
from setuptools import find_packages, setup

package_name = "urrc_track_gazebo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
        ("share/" + package_name + "/meshes", glob("meshes/*")),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/docs", glob("docs/*")),
        ("share/" + package_name + "/scripts", glob("scripts/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="URRC Team",
    maintainer_email="student@example.com",
    description="1/10 F1-inspired LiDAR cone racing circuits for Gazebo Harmonic",
    license="MIT",
    extras_require={"test": ["pytest"]},
)
