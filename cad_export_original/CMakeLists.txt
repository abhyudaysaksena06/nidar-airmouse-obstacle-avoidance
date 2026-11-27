cmake_minimum_required(VERSION 2.8.3)

project(FINAL_TWO_PLATE_ASSEMBLY_(MINUS_ESC)_urdfff.SLDASM)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
