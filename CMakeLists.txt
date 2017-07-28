#Author: Kurt Meister
cmake_minimum_required(VERSION 2.8.3)
project(meister)
add_definitions(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  rosbag
  laser_geometry
)

find_package(Boost REQUIRED COMPONENTS thread)

set(SOURCE_FILES
        src/ObstacleDetector.cpp
        include/meister/ObstacleDetector.h
        src/DetectionGrid.cpp
        include/meister/DetectionGrid.h
        src/LaserDataProcessor.cpp
        include/meister/LaserDataProcessor.h
        src/LidarFilterNode.cpp
        src/NodeController.cpp
        include/meister/NodeController.h
        src/ObstacleDetectorNode.cpp
        src/StateUpdate.cpp
        include/meister/StateUpdate.h
        src/StateUpdateNode.cpp
        src/VehicleStatusMonitor.cpp
        include/meister/VehicleStatusMonitor.h
        src/VehicleStatusMonitorNode.cpp
        src/WorldBuilder.cpp
        include/meister/WorldBuilder.h
        )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
   INCLUDE_DIRS include/meister/
#   LIBRARIES meister
   CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs rosbag laser_geometry
   DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  include/meister
)

## Declare a C++ library
add_library(NodeController
        src/NodeController.cpp
        )
add_library(StateUpdate
        src/StateUpdate.cpp
        )

add_library(ObstacleDetector
   src/ObstacleDetector.cpp
   src/DetectionGrid.cpp
   src/WorldBuilder.cpp
 )

add_library(LaserData
    src/LaserDataProcessor.cpp
)

add_library(LidarFilter
   src/LidarFilter.cpp
)

add_library(VehicleStatusMonitor
    src/VehicleStatusMonitor.cpp
)


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(meister ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(meister_node src/meister_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(meister_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(meister_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(ObstacleDetector NodeController ${catkin_LIBRARIES})
target_link_libraries(NodeController ${catkin_LIBRARIES})
add_executable(state_update_node src/StateUpdateNode.cpp)
target_link_libraries(state_update_node NodeController StateUpdate ${catkin_LIBRARIES})
add_executable(lidar_filter src/LidarFilterNode.cpp)
target_link_libraries(lidar_filter LidarFilter NodeController ${catkin_LIBRARIES})
add_executable(obstacle_detector src/ObstacleDetectorNode.cpp)
target_link_libraries(obstacle_detector ObstacleDetector NodeController ${catkin_LIBRARIES})
add_executable(laser_data src/LaserDataNode.cpp )
target_link_libraries(laser_data LaserData ObstacleDetector NodeController ${catkin_LIBRARIES})
add_executable(vehicle_status_monitor src/VehicleStatusMonitorNode.cpp)
target_link_libraries(vehicle_status_monitor  VehicleStatusMonitor ObstacleDetector NodeController ${catkin_LIBRARIES})


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
# install(TARGETS meister meister_node
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
catkin_add_gtest(detection_grid_test test/DetectionGridTests.cpp)
if(TARGET detection_grid_test)
  target_link_libraries(detection_grid_test ObstacleDetector ${catkin_LIBRARIES})
endif()
catkin_add_gtest(lidar_filter_test test/LidarFilterNodeTest.cpp)
if(TARGET lidar_filter_test)
    target_link_libraries(lidar_filter_test LidarFilter NodeController ${catkin_LIBRARIES})
endif()
catkin_add_gtest(obstacle_detector_test test/ObstacleDetectorNodeTest.cpp)
if(TARGET obstacle_detector_test)
    target_link_libraries(obstacle_detector_test ObstacleDetector NodeController ${catkin_LIBRARIES})
endif()
catkin_add_gtest(vehicle_status_monitor_test test/VehicleStatusMonitorNodeTest.cpp)
if(TARGET vehicle_status_monitor_test)
    target_link_libraries(vehicle_status_monitor_test VehicleStatusMonitor ObstacleDetector NodeController ${catkin_LIBRARIES})
endif()
catkin_add_gtest(state_update_test test/StateUpdateNodeTest.cpp)
if(TARGET state_update_test)
    target_link_libraries(state_update_test StateUpdate NodeController ${catkin_LIBRARIES})
endif()
catkin_add_gtest(sdf_model_test test/SDFModelTest.cpp)
if(TARGET sdf_model_test)
    target_link_libraries(sdf_model_test ${catkin_LIBRARIES})
endif()
catkin_add_gtest(system_integration_test test/SystemIntegrationTest.cpp)
if(TARGET system_integration_test)
    target_link_libraries(system_integration_test ${catkin_LIBRARIES})
endif()
catkin_add_gtest(laser_data_test test/LaserDataNodeTest.cpp)
if(TARGET laser_data_test)
    target_link_libraries(laser_data_test LaserData NodeController ${catkin_LIBRARIES})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
