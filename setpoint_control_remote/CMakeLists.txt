cmake_minimum_required(VERSION 2.8.3)
project(setpoint_control_remote)


find_package(catkin REQUIRED COMPONENTS
  mavros_msgs
  roscpp
  std_msgs
  actionlib
  actionlib_msgs
  message_generation
  geometry_msgs
  tf
  pcl_ros
  pcl_conversions
  pcl_msgs
  sensor_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate actions in the 'action' folder
 add_action_files(
    FILES
	DIRECTORY action
    FILES setpoint_control_commands.action
    take_cloud_snapshoot.action
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
CATKIN_DEPENDS actionlib_msgs pcl_ros sensor_msgs pcl_conversions pcl_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(setpoint_control_remote src/setpoint_remote.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(setpoint_control_remote ${setpoint_control_remote_EXPORTED_TARGETS} )

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)




include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


#############
## Install ##
#############



#############
## Testing ##
#############

