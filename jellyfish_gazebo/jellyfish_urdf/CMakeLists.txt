cmake_minimum_required(VERSION 2.8.3)
project(jellyfish_urdf)

find_package(catkin REQUIRED COMPONENTS
	gazebo_ros
	rviz_visual_tools
	tf2
	tf2_ros
	tf_conversions
	rospy
	dji_sdk
	message_generation
	geometry_msgs
	marvelmind_nav
)

add_message_files(
  FILES
  hedge_pos.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
	CATKIN_DEPENDS 
	joint_state_controller
	robot_state_publisher
	dji_m210_controllers_gazebo
	rotors_gazebo_plugins
	rospy
	mav_msgs
	wave_gazebo
	message_runtime
)

install(DIRECTORY
launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS src/scripts/gazebo_drone_movement.py src/scripts/fake_dji_sdk.py src/scripts/fake_marvelmind.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)
install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)