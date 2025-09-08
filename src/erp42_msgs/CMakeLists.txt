# --------------------------------------------------------------------------------------------------
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2025 Minkyu Kil
# --------------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.5)
project(erp42_msgs)

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
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --------------------------------------------------------------------------------------------------
# Generate ROS2 message interfaces
# --------------------------------------------------------------------------------------------------
rosidl_generate_interfaces(
    ${PROJECT_NAME}
        "msg/ControlCommand.msg"
        "msg/Feedback.msg"
        "srv/ModeCommand.srv"

    DEPENDENCIES
        builtin_interfaces 
        std_msgs
)

# --------------------------------------------------------------------------------------------------
# Export generated message runtime dependencies
# --------------------------------------------------------------------------------------------------
ament_export_dependencies(rosidl_default_runtime)

# --------------------------------------------------------------------------------------------------
# Register the package with ament
# --------------------------------------------------------------------------------------------------
ament_package()