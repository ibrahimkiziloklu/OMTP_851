cmake_minimum_required(VERSION 3.0.2)
project(omtp_lecture3)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

cmake_policy(SET CMP0054 NEW)

#if(NOT WIN32)
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
#endif()

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  gazebo_msgs
  gazebo_plugins
  gazebo_ros
  geometry_msgs
  message_generation
  roscpp
  std_msgs
  std_srvs
  tf
)

find_package(gazebo REQUIRED)
ign_import_target(UUID)
find_package(TinyXML REQUIRED)

########################
## Message generation ##
########################

add_message_files(
  DIRECTORY
    msg
  FILES
    Model.msg
    LogicalCameraImage.msg
)

#add_service_files(
#  DIRECTORY
#    srv
#  FILES
#)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
  CATKIN_DEPENDS
    gazebo_msgs
    gazebo_plugins
    gazebo_ros
    geometry_msgs
    message_runtime
    roscpp
    std_msgs
    std_srvs
    tf
  DEPENDS
    TinyXML
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

catkin_install_python(PROGRAMS scripts/transform_object_pose.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

###########
## Build ##
###########

# Create the libROSLogicalCameraPlugin.so library.
set(ros_logical_camera_plugin_name ROSLogicalCameraPlugin)
add_library(${ros_logical_camera_plugin_name} src/plugins/ROSLogicalCameraPlugin.cc)
target_link_libraries(${ros_logical_camera_plugin_name}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)
add_dependencies(${ros_logical_camera_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${ros_logical_camera_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
