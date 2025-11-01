# --------------------------------------------------------------------------------------------------
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2025 Minkyu Kil
# --------------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.22)
project(erp42_serial)

# --------------------------------------------------------------------------------------------------
# Find required packages
# --------------------------------------------------------------------------------------------------
find_package(erp42_cmake REQUIRED)
erp42_package()

# --------------------------------------------------------------------------------------------------
# Add library and executables
# --------------------------------------------------------------------------------------------------
ament_auto_add_library(${PROJECT_NAME} SHARED
    src/serial_port.cpp
    src/serial_bridge.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "erp42_serial::SerialBridge"
    EXECUTABLE serial_bridge_composable
)

ament_auto_add_executable(serial_bridge
    node/serial_bridge_node.cpp
)

target_link_libraries(serial_bridge
    ${PROJECT_NAME}
)

# --------------------------------------------------------------------------------------------------
# Install package
# --------------------------------------------------------------------------------------------------
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)

install(
    DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)