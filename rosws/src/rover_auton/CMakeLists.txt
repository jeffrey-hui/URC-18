cmake_minimum_required(VERSION 2.8.3)
project(rover_auton)

find_package(catkin REQUIRED COMPONENTS
        control_toolbox
        geometry_msgs
        nav_msgs
        roscpp
        rospy
        std_msgs
        tf2
        tf2_ros
        actionlib_msgs
)



## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )


###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



add_executable(rover src/main.cpp src/rover.cpp src/rover.h)
target_link_libraries(rover ${catkin_LIBRARIES} tinyxml)
