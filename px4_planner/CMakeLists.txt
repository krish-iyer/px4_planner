cmake_minimum_required(VERSION 3.0.2)
project(px4_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  visualization_msgs
  message_generation
  rospy
  std_srvs
  mavros_msgs
  sensor_msgs
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

add_service_files(
  FILES
  target.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   std_srvs
   sensor_msgs
)
# catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aepl_planner
 CATKIN_DEPENDS 
    geometry_msgs 
    rospy 
    sensor_msgs 
    std_msgs 
    mavros_msgs
    visualization_msgs
    message_generation
	  message_runtime
#  DEPENDS system_lib
)

include_directories( include ${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/planner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
scripts/planner.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

