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

find_package(ros_environment REQUIRED)

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

find_package(OpenCV 4 REQUIRED)

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
add_executable(mynteye_wrapper_node_2024 src/node3.cpp src/Matrix2.cpp)

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

target_include_directories(mynteye_wrapper_node_2024
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mynteye_wrapper_node_2024
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
target_link_libraries(mynteye_wrapper_node_2024 
  mynteye 
)
install(TARGETS 
mynteye_wrapper_node_2024
 DESTINATION lib/${PROJECT_NAME}
)

ament_package()
