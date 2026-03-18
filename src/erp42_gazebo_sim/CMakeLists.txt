# --------------------------------------------------------------------------------------------------
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2025 Minkyu Kil
# --------------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.22)
project(erp42_gazebo_sim)

# --------------------------------------------------------------------------------------------------
# Find required packages
# --------------------------------------------------------------------------------------------------
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
find_package(erp42_cmake REQUIRED)
erp42_package()

# --------------------------------------------------------------------------------------------------
# ERP42 control plugin (Gazebo gz-sim system plugin with direct ROS 2 interface)
# --------------------------------------------------------------------------------------------------
ament_auto_add_library(erp42_control SHARED
    plugins/erp42_control.cpp
)

target_link_libraries(erp42_control
    gz-sim8::gz-sim8
    gz-plugin2::register
)

# --------------------------------------------------------------------------------------------------
# Install package
# --------------------------------------------------------------------------------------------------
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)

install(
    DIRECTORY config launch rviz meshes urdf worlds
    DESTINATION share/${PROJECT_NAME}
)