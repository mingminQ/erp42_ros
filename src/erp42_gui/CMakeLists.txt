# --------------------------------------------------------------------------------------------------
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2025 Minkyu Kil
# --------------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.8)
project(erp42_gui)

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
# ...
# --------------------------------------------------------------------------------------------------
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

# --------------------------------------------------------------------------------------------------
# Find required ROS2 and message packages
# --------------------------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(erp42_util REQUIRED)
find_package(erp42_msgs REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)

# --------------------------------------------------------------------------------------------------
# Include local header files
# --------------------------------------------------------------------------------------------------
include_directories(
    include
)

# --------------------------------------------------------------------------------------------------
# ERP42 ROS2 QT5 Feedback Monitor
# --------------------------------------------------------------------------------------------------
set(ERP42_FEEDBACK_MONITOR_SOURCE
    include/erp42_gui/feedback_monitor.hpp
    src/feedback_monitor.cpp
    ui/feedback_monitor.ui
)
add_executable(feedback_monitor
    node/feedback_monitor_node.cpp
    ${ERP42_FEEDBACK_MONITOR_SOURCE}
)
ament_target_dependencies(feedback_monitor
    rclcpp
    erp42_util
    erp42_msgs
)
target_include_directories(feedback_monitor
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(feedback_monitor
    Qt5::Core
    Qt5::Widgets
)
install(
    TARGETS feedback_monitor
    DESTINATION lib/${PROJECT_NAME}
)

# --------------------------------------------------------------------------------------------------
# ERP42 ROS2 QT5 GUI controller
# --------------------------------------------------------------------------------------------------
set(ERP42_GUI_CONTROLLER_SOURCE
    include/erp42_gui/controller.hpp
    src/controller.cpp
    ui/controller.ui
)
add_executable(controller
    node/controller_node.cpp
    ${ERP42_GUI_CONTROLLER_SOURCE}
    ${ERP42_FEEDBACK_MONITOR_SOURCE}
)
ament_target_dependencies(controller
    rclcpp
    erp42_util
    erp42_msgs
)
target_include_directories(controller
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(controller
    Qt5::Core
    Qt5::Widgets
)
install(
    TARGETS controller
    DESTINATION lib/${PROJECT_NAME}
)

# --------------------------------------------------------------------------------------------------
# Register the package with ament
# --------------------------------------------------------------------------------------------------
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_package()