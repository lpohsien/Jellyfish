cmake_minimum_required(VERSION 2.8.3)
project(dji_m210_description)

find_package(catkin REQUIRED COMPONENTS
	gazebo_ros
	rviz_visual_tools
)

find_package(gazebo REQUIRED)
	catkin_package(
)

catkin_package(
	CATKIN_DEPENDS 
	dji_m210_controllers_gazebo
	rotors_gazebo_plugins
)
install(DIRECTORY meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)