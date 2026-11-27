"""Shared helper: build robot_description from the xacro."""
import os
from ament_index_python.packages import get_package_share_directory
import xacro


def robot_description(px4=False):
    share = get_package_share_directory("nidar_drone_description")
    doc = xacro.process_file(
        os.path.join(share, "urdf", "nidar_drone.urdf.xacro"),
        mappings={"mesh_uri": "file://" + os.path.join(share, "meshes"),
                  "px4": "true" if px4 else "false"})
    return doc.toxml()
