cmake_minimum_required(VERSION 2.8.3)
project(dji_m210_controllers_gazebo)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  gazebo_ros
  gazebo_dev
  gazebo_ros_control
)
include_directories(include ${catkin_INCLUDE_DIRS} /usr/include/gazebo-9/)

catkin_package(
  CATKIN_DEPENDS roscpp
  std_msgs
  sensor_msgs
  gazebo_ros
)

### Create HITL Library.
add_library(dji_m210_controller_gazebo_hitl SHARED src/dji_m210_ros_hitl_controller_gazebo.cpp)
target_link_libraries(dji_m210_controller_gazebo_hitl ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
install(TARGETS dji_m210_controller_gazebo_hitl
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)
