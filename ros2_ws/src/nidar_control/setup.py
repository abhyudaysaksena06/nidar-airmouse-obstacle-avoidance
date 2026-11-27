from glob import glob
from setuptools import setup

package_name = "nidar_control"
setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="NIDAR Controls",
    description="PX4 offboard mission and PID logger for the NIDAR drone",
    license="BSD-3-Clause",
    entry_points={"console_scripts": [
        "offboard_control = nidar_control.offboard_control:main",
        "pid_logger = nidar_control.pid_logger:main",
        "ev_bridge = nidar_control.ev_bridge:main",
    ]},
)
