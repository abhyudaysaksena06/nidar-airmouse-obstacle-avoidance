# Original SolidWorks export (reference only)

The files exactly as the SolidWorks-to-URDF exporter produced them (meshes are in
`ros2_ws/src/nidar_drone_description/meshes`, unchanged; the 7 MB exporter log was not kept).
This ROS 1 package is **not** used directly. `tools/cad_urdf_to_xacro.py` turns
`urdf/original_export.urdf` into `ros2_ws/src/nidar_drone_description/urdf/nidar_drone_body.xacro`.
See docs/CONTROLS_GUIDE.md, section 10, for the problems found in this export.
