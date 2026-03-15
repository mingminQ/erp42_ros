# --------------------------------------------------------------------------------------------------
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2025 Minkyu Kil
# --------------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.22)
project(erp42_gazebo)

# --------------------------------------------------------------------------------------------------
# Find required packages
# --------------------------------------------------------------------------------------------------
find_package(erp42_cmake REQUIRED)
erp42_package()

# --------------------------------------------------------------------------------------------------
# Add library and executables
# --------------------------------------------------------------------------------------------------
ament_auto_add_library(${PROJECT_NAME} SHARED
    src/gazebo_bridge.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "erp42_gazebo::GazeboBridge"
    EXECUTABLE gazebo_bridge_composable
)

ament_auto_add_executable(gazebo_bridge
    node/gazebo_bridge_node.cpp
)

target_link_libraries(gazebo_bridge
    ${PROJECT_NAME}
)

# --------------------------------------------------------------------------------------------------
# Install package
# --------------------------------------------------------------------------------------------------
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)

install(
    DIRECTORY config launch rviz urdf worlds
    DESTINATION share/${PROJECT_NAME}/
)