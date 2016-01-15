cmake_minimum_required(VERSION 2.8.3)
project(laser_condition)

find_package(catkin REQUIRED COMPONENTS
	roscpp 
	std_msgs 
	sensor_msgs
)

find_package(Boost REQUIRED COMPONENTS)

catkin_package(
#  INCLUDE_DIRS
   LIBRARIES laser_condition
   CATKIN_DEPENDS roscpp std_msgs sensor_msgs
#   DEPENDS
)

## Declare a cpp library
# add_library(laser_condition
#   src/${PROJECT_NAME}/laser_condition.cpp
# )

## Declare a cpp executable
add_executable(laser_condition_node src/laser_condition.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(laser_condition_node)

## Specify libraries to link a library or executable target against
target_link_libraries(laser_condition_node ${catkin_LIBRARIES}
)