cmake_minimum_required(VERSION 2.8.3)
project(dank_auton)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
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
set(CMAKE_CXX_STANDARD 14)
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



add_executable(dank_rover src/jank.cpp)
target_link_libraries(dank_rover ${catkin_LIBRARIES} tinyxml)
