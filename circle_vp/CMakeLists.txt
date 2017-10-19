cmake_minimum_required(VERSION 2.8.3)
project(circle_vp)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  tf_conversions
  actionlib
  actionlib_msgs
  message_generation
  geometry_msgs
)

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS system)
 
 
################################################
## Declare ROS messages, services and actions ##
################################################

## Generate actions in the 'action' folder
 add_action_files(FILES 
   circular_view_points.action
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   actionlib_msgs std_msgs geometry_msgs
 )



###################################
## catkin specific configuration ##
###################################


catkin_package(
  LIBRARIES circle_vp
  CATKIN_DEPENDS roscpp std_msgs tf actionlib_msgs
)

###########
## Build ##
###########

add_executable(circle_vp src/circle_vp.cpp)
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
find_package(Eigen3 REQUIRED) 
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(
   ${Eigen_INCLUDE_DIRS}
 )

 target_link_libraries(circle_vp
   ${Eigen_LIBRARIES}
 )
 
 add_dependencies(
  circle_vp
  ${circle_vp_EXPORTED_TARGETS}
)
add_dependencies(circle_vp ${catkin_EXPORTED_TARGETS})



target_link_libraries(circle_vp
${catkin_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############

