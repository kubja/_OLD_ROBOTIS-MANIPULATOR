cmake_minimum_required(VERSION 2.8.3)
project(robotis_manipulator_h_calc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  control_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  moveit_msgs
  moveit_core 
  moveit_ros_planning 
  moveit_ros_planning_interface
  pluginlib
  robotis_controller_msgs 
  cmake_modules
)

find_package(Eigen REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  roscpp
  control_msgs 
  sensor_msgs 
  std_msgs 
  geometry_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
  interactive_markers
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(moveit_node 
	src/main_moveit.cpp
	src/base/callback.cpp
	src/base/jointstate.cpp
	src/base/process.cpp
	src/base/trajectory.cpp
)

add_executable(kinematics_node 
	src/main_kinematics.cpp
	src/base/callback.cpp
	src/base/jointstate.cpp
	src/base/process.cpp
	src/base/trajectory.cpp
)

add_dependencies(moveit_node main_generate_messages_cpp)
add_dependencies(kinematics_node main_generate_messages_cpp)

target_link_libraries(moveit_node
  ${catkin_LIBRARIES}
  ${ROSRT_PLATFORM_LDFLAGS}
)

target_link_libraries(kinematics_node
  ${catkin_LIBRARIES}
  ${ROSRT_PLATFORM_LDFLAGS}
)

