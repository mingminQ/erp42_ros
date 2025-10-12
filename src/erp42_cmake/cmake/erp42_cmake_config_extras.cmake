# --------------------------------------------------------------------------------------------------
# CMake configuration for erp42_ros package
# --------------------------------------------------------------------------------------------------
macro(erp42_package)

    # Ensure C++17 standard is used if not already set
    if(NOT CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD 17)
    endif()

    # Add extra warning flags for GCC and Clang
    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    # Find required ROS2 and message packages
    find_package(ament_cmake_auto REQUIRED)
    ament_auto_find_build_dependencies()

    # Set ROS_DISTRO macros
    add_compile_definitions($ENV{ROS_DISTRO})

    # Test dependencies
    if(BUILD_TEST_SOURCES)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
    endif()

endmacro()