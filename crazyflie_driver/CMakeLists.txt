cmake_minimum_required(VERSION 2.8.3)
project(crazyflie_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  tf
  crazyflie_cpp
)
# Enable C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#SET(CMAKE_CXX_COMPILER             "gcc-4.9")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AddCrazyflie.srv
  GoTo.srv
  Land.srv
  RemoveCrazyflie.srv
  SetGroupMask.srv
  StartTrajectory.srv
  Stop.srv
  Takeoff.srv
  UpdateParams.srv
  UploadTrajectory.srv
  sendPacket.srv
)

add_message_files(
  FILES
  LogBlock.msg
  GenericLogData.msg
  FullState.msg
  TrajectoryPolynomialPiece.msg
  crtpPacket.msg
  Hover.msg
  Position.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
   std_msgs
   geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES crazyflie
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    tf
    crazyflie_cpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(crazyflie_server
  src/crazyflie_server.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(crazyflie_server
  crazyflie_driver_generate_messages_cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(crazyflie_server
  ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(crazyflie_add
  src/crazyflie_add.cpp
)

add_dependencies(crazyflie_add
  crazyflie_driver_generate_messages_cpp
)

target_link_libraries(crazyflie_add
  ${catkin_LIBRARIES}
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
# install(TARGETS crazyflie crazyflie_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_crazyflie.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
