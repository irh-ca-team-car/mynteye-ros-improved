# Copyright 2018 Slightech Co., Ltd. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(mynt_eye_ros_wrapper_for_xavier)


set(CMAKE_PREFIX_PATH /opt/opencv341/lib;${CMAKE_PREFIX_PATH})

find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# packages
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wall -Wextra -Wfatal-errors)
endif()

function(checkPackage package customMessage)
  set(varName "${package}_FOUND")
  if (NOT "${${varName}}")
    string(REPLACE "_" "-" aptPackage ${package})
    if("${customMessage}" STREQUAL "")
      message(FATAL_ERROR "\n\n ${package} is missing, please try to install it with:\n sudo apt-get install ros-$(rosversion -d)-${aptPackage}\n\n")
    else()
      message(FATAL_ERROR "\n\n ${customMessage} \n\n")
    endif()
  endif()
endfunction(checkPackage)

find_package(OpenCV 3 REQUIRED)

if(${ROS_VERSION} EQUAL 1)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  message_generation
  nodelet
  roscpp
  sensor_msgs
  std_msgs
  tf
)

checkPackage("cv_bridge" "")
checkPackage("geometry_msgs" "")
checkPackage("image_transport" "")
checkPackage("message_generation" "")
checkPackage("nodelet" "")
checkPackage("roscpp" "")
checkPackage("sensor_msgs" "")
checkPackage("std_msgs" "")
checkPackage("tf" "")

## messages

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport message_runtime nodelet roscpp sensor_msgs std_msgs tf
)

#get_filename_component(SDK_DIR "${PROJECT_SOURCE_DIR}/../../../.." ABSOLUTE)
#LIST(APPEND CMAKE_PREFIX_PATH ${SDK_DIR}/_install/lib/cmake)

LIST(APPEND CMAKE_MODULE_PATH cmake)

find_package(mynteye REQUIRED)
message(STATUS "Found mynteye: ${mynteye_VERSION}")

if(NOT mynteye_WITH_API)
  message(FATAL_ERROR "Must with API layer :(")
endif()


if(mynteye_WITH_GLOG)
  find_package(glog REQUIRED)
endif()

# targets

add_compile_options(-std=c++11)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${SDK_DIR}/src
  include/
)

set(LINK_LIBS
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  mynteye
)

#target_link_libraries(mynteye_wrapper ${LINK_LIBS})
#add_dependencies(mynteye_wrapper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(mynteye_wrapper_node src/node.cpp src/Matrix.cpp)
add_dependencies(mynteye_wrapper_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mynteye_wrapper_node ${LINK_LIBS})

if(MSVC OR MSYS OR MINGW)
  target_compile_definitions(mynteye_wrapper_node
    PUBLIC GLOG_NO_ABBREVIATED_SEVERITIES
  )
endif()

# install

#install(PROGRAMS
#  scripts/get_device_info.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(TARGETS mynteye_wrapper_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

elseif(${ROS_VERSION} EQUAL 2)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(imu_filter_madgwick REQUIRED)

LIST(APPEND CMAKE_MODULE_PATH cmake)
find_package(mynteye REQUIRED)
message(STATUS "Found mynteye: ${mynteye_VERSION}")

add_executable(mynteye_wrapper_node src/node2.cpp src/Matrix2.cpp)

target_include_directories(mynteye_wrapper_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mynteye_wrapper_node
  cv_bridge 
  geometry_msgs 
  image_transport 
  rclcpp 
  sensor_msgs 
  std_msgs 
  tf2
  tf2_ros
  mynteye
  OpenCV
  imu_filter_madgwick
)
target_link_libraries(mynteye_wrapper_node 
  mynteye 
)
install(TARGETS 
 mynteye_wrapper_node
 DESTINATION lib/${PROJECT_NAME}
)

ament_package()
endif()