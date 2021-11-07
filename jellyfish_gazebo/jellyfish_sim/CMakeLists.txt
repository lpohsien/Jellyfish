cmake_minimum_required(VERSION 2.8.3)
project(jellyfish_sim)

find_package(catkin REQUIRED COMPONENTS
	gazebo_ros
	rviz_visual_tools
	tf2
	rospy
	dji_sdk
	message_generation
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
)

catkin_package(
	CATKIN_DEPENDS 
	joint_state_controller
	robot_state_publisher
	dji_m210_controllers_gazebo
	rotors_gazebo_plugins
	tf2
	rospy
	mav_msgs
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