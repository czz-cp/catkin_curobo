cmake_minimum_required(VERSION 3.0.2)
project(robot_arm_planner)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  trajectory_msgs
  tf
  tf2_ros
  visualization_msgs
  std_srvs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    trajectory_msgs
    tf
    tf2_ros
    visualization_msgs
    std_srvs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark executables for installation
install(PROGRAMS
  scripts/curobo_planner.py
  scripts/ur10e_controller.py
  scripts/navrl_adapter.py
  scripts/safe_action_planner.py
  scripts/goal_publisher.py
  scripts/arm_pose_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN "*.launch"
)

## Mark config files for installation
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)