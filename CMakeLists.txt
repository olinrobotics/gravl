cmake_minimum_required(VERSION 2.8.11)
project(gravl)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "-std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  image_transport
  dynamic_reconfigure
  ackermann_msgs
  message_generation
  mavros_msgs
  nav_msgs
  tf
  tf2
  tf2_geometry_msgs
  cv_bridge
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# OpenCV dependencies
find_package(OpenCV 3.2 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

## QT dependencies
find_package(Qt5Widgets)

IF(Qt5Widgets_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
  set(CMAKE_AUTOMOC ON)
  set(CMAKE_AUTORCC ON)
  set(CMAKE_AUTOUIC ON)

  find_package(Qt5Qml)
  find_package(Qt5Quick)
  find_package(Qt5Positioning)
ENDIF()


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Hemisphere.msg
  RoboClawStats.msg
  ImuSafety.msg
  TwistLabeled.msg
)

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
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
   cfg/gravl_cfg.cfg
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
  INCLUDE_DIRS /usr/include/boost/algorithm
#  LIBRARIES tractor
#  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS message_runtime system_lib lrt
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${QtQml_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/tractor.cpp
# )

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable( Camera src/Camera/Camera.cpp )
add_executable( Teleop src/Teleop/Teleop.cpp )
add_executable( DriveState src/DriveState/DriveState.cpp )
add_executable( MainState src/MainState/MainState.cpp)
add_executable( Driver src/Driver/Driver.cpp )
add_executable( Waypoint src/Waypoint/Waypoint.cpp )
add_executable( Hemisphere src/Hemisphere/Hemisphere.cpp )
add_executable( idk src/arm_drone.cpp )
add_executable( road_detection src/road_detection.cpp )
add_executable(imu_safety src/imu_safety/ImuSafety.cpp)
add_executable(imu_base src/ImuBase/ImuBase.cpp)
add_executable(Watchdog src/Watchdog/Watchdog.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries( Camera ${OpenCV_LIBS} ${catkin_LIBRARIES} )
target_link_libraries( Teleop ${catkin_LIBRARIES} )
target_link_libraries( DriveState ${catkin_LIBRARIES} )
target_link_libraries( MainState ${catkin_LIBRARIES} )
target_link_libraries( Driver ${catkin_LIBRARIES} )
target_link_libraries( Waypoint ${catkin_LIBRARIES} )
target_link_libraries( Hemisphere ${catkin_LIBRARIES} ${Boost_LIBRARIES} -lrt )
target_link_libraries( idk ${catkin_LIBRARIES} )
target_link_libraries( road_detection ${OpenCV_LIBS} )
target_link_libraries(imu_safety ${catkin_LIBRARIES})
target_link_libraries(imu_base ${catkin_LIBRARIES})
target_link_libraries(Watchdog ${catkin_LIBRARIES})


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(Hemisphere ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(imu_safety ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#IF(Qt5Widgets_FOUND)
#  MESSAGE(WARNING ${Qt5Widgets_INCLUDE_DIRS})
#  qt5_add_resources(QTEST_RESOURCES src/QT/qml.qrc)
#  add_executable( gps_map src/QT/main.cpp src/QT/tractor_controller.cpp ${QTEST_RESOURCES} )
#  add_dependencies( gps_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
#  target_link_libraries( gps_map
#    Qt5::Widgets
#    Qt5::Qml
#    Qt5::Quick
#    Qt5::Positioning
#    ${catkin_LIBRARIES} )
#ENDIF()

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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tractor.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
