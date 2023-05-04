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
project(mynt_eye_ros_wrapper)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# packages

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(builtin_interfaces REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mynt_eye_ros_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
include_directories(${Boost_INCLUDE_DIRS})

set(dependencies 
  cv_bridge
  geometry_msgs
  image_transport
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  mynt_eye_ros_msgs
  tf2
  rosidl_default_generators
  builtin_interfaces
)

## messages

#get_filename_component(SDK_DIR "${PROJECT_SOURCE_DIR}/../../../.." ABSOLUTE)
#LIST(APPEND CMAKE_PREFIX_PATH ${SDK_DIR}/_install/lib/cmake)

LIST(APPEND CMAKE_MODULE_PATH cmake)

find_package(mynteye REQUIRED)
message(STATUS "Found mynteye: ${mynteye_VERSION}")

if(NOT mynteye_WITH_API)
  message(FATAL_ERROR "Must with API layer :(")
endif()

find_package(OpenCV REQUIRED)

if(mynteye_WITH_GLOG)
  find_package(glog REQUIRED)
endif()

# targets

add_compile_options(-std=c++17)

include_directories(
  ${SDK_DIR}/src
)

set(LINK_LIBS
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  mynteye
)

# add_library(mynteye_wrapper src/wrapper_nodelet.h src/wrapper_nodelet.cc)
# target_link_libraries(mynteye_wrapper ${LINK_LIBS} )


# ament_target_dependencies(mynteye_wrapper
#   ${dependencies}
# )

add_executable(mynteye_wrapper_node src/wrapper_node.cc src/wrapper_nodelet.cc)
target_link_libraries(mynteye_wrapper_node ${LINK_LIBS})

ament_target_dependencies(mynteye_wrapper_node
  ${dependencies}
)

# install

#install(PROGRAMS
#  scripts/get_device_info.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(TARGETS mynteye_wrapper_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY rviz config launch mesh
  DESTINATION share/${PROJECT_NAME}/
)

# install(FILES nodelet_plugins.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

ament_export_dependencies(${dependencies} rosidl_default_runtime)
ament_package()