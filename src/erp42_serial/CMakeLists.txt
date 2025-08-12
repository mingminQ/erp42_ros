# --------------------------------------------------------------------------------------------------
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2025 Minkyu Kil
# --------------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.8)
project(erp42_serial)

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
find_package(erp42_msgs REQUIRED)
find_package(erp42_util REQUIRED)

# --------------------------------------------------------------------------------------------------
# Include local header files
# --------------------------------------------------------------------------------------------------
include_directories(
    include
)

# --------------------------------------------------------------------------------------------------
# ERP42 racing ROS2 serial driver
# --------------------------------------------------------------------------------------------------
add_executable(serial_bridge
    node/serial_bridge_node.cpp
    src/serial_bridge.cpp
    src/serial_port.cpp
)
ament_target_dependencies(serial_bridge
    rclcpp
    erp42_msgs
    erp42_util
)
target_include_directories(serial_bridge 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
install(
    TARGETS serial_bridge
    DESTINATION lib/${PROJECT_NAME}
)

# --------------------------------------------------------------------------------------------------
# Install launch files
# --------------------------------------------------------------------------------------------------
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# --------------------------------------------------------------------------------------------------
# Register the package with ament
# --------------------------------------------------------------------------------------------------
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_package()