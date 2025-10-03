# --------------------------------------------------------------------------------------------------
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2025 Minkyu Kil
# --------------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.22)
project(erp42_gazebo)

# --------------------------------------------------------------------------------------------------
# Ensure C++17 standard is used if not already set
# --------------------------------------------------------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# --------------------------------------------------------------------------------------------------
# Add extra warning flags for GCC and Clang
# --------------------------------------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------------------------------------------------------
# Find required ROS2 and message packages
# --------------------------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(erp42_msgs REQUIRED)
find_package(erp42_util REQUIRED)
find_package(erp42_description REQUIRED)

# --------------------------------------------------------------------------------------------------
# ERP42 ROS2 gazebo bridge
# --------------------------------------------------------------------------------------------------
add_executable(gazebo_bridge
    node/gazebo_bridge_node.cpp
    src/gazebo_bridge.cpp
)

ament_target_dependencies(gazebo_bridge
    rclcpp
    nav_msgs
    sensor_msgs
    geometry_msgs
    gazebo_msgs
    erp42_msgs
    erp42_util
    erp42_description
)

target_include_directories(gazebo_bridge 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

install(
    TARGETS gazebo_bridge
    DESTINATION lib/${PROJECT_NAME}
)

# --------------------------------------------------------------------------------------------------
# Install launch files, URDF, and world files
# --------------------------------------------------------------------------------------------------
install(
    DIRECTORY config launch rviz urdf worlds
    DESTINATION share/${PROJECT_NAME}/
)

# --------------------------------------------------------------------------------------------------
# Register the package with ament
# --------------------------------------------------------------------------------------------------
ament_package()