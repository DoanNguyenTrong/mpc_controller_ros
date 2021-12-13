cmake_minimum_required(VERSION 3.5)
project(mpc_controller_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11 -O3)

# options for build configuration
option(BUILD_EXAMPLE "Whether or not building the CppAD & Ipopt example" OFF) 

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  #ackermann_msgs
  pluginlib
  message_generation
  move_base
  base_local_planner
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( ${catkin_INCLUDE_DIRS} include )
include_directories(/usr/include)
link_directories(/usr/lib)



catkin_package(
   INCLUDE_DIRS include
   LIBRARIES mpc_ros
   CATKIN_DEPENDS costmap_2d dynamic_reconfigure geometry_msgs move_base roscpp rospy std_msgs tf visualization_msgs pluginlib
#  DEPENDS system_lib
)

###########
## Build ##
###########

# General MPC_Node 
ADD_EXECUTABLE( mpc_controller_node src/mpc_controller.cpp src/mpc_controller_node.cpp )
TARGET_LINK_LIBRARIES(mpc_controller_node ipopt ${catkin_LIBRARIES} )


#############
## Example ##
#############
if(BUILD_EXAMPLE)
    ADD_EXECUTABLE( CppAD_started example/CppAD_started.cpp )
    TARGET_LINK_LIBRARIES(CppAD_started)
    
    ADD_EXECUTABLE( CppAD_Ipopt example/CppAD_Ipopt.cpp )
    TARGET_LINK_LIBRARIES(CppAD_Ipopt ipopt)
endif(BUILD_EXAMPLE)
