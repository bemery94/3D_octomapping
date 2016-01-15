cmake_minimum_required(VERSION 2.8.3)
project(myahrs_plus_imu)

find_package(catkin REQUIRED 
	roscpp 
	std_msgs 
	tf
)

find_package(Boost REQUIRED COMPONENTS 
	system
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES myahrs_plus_imu
  CATKIN_DEPENDS
  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
# add_library(myahrs_plus_imu
#   src/${PROJECT_NAME}/myahrs_plus_imu.cpp
# )

## Declare a cpp executable
add_executable(myahrs_plus_imu_node src/myahrs_plus_imu.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(myahrs_plus_imu_node myahrs_plus_imu_generate_messages_cpp tf)

## Specify libraries to link a library or executable target against
target_link_libraries(myahrs_plus_imu_node ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS myahrs_plus_imu myahrs_plus_imu_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_myahrs_plus_imu.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
