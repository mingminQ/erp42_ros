# --------------------------------------------------------------------------------------------------
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2025 Minkyu Kil
# --------------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.22)
project(erp42_cmake NONE)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS
    "cmake/erp42_cmake_config_extras.cmake"
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
    INSTALL_TO_SHARE cmake
)