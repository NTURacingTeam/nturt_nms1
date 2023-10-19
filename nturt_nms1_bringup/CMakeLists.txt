cmake_minimum_required(VERSION 3.8)
project(nturt_nms1_bringup)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_msgs REQUIRED)

add_executable(ackermann_steering_controller_remapper_node
        app/ackermann_steering_controller_remapper_node.cpp
)
ament_target_dependencies(ackermann_steering_controller_remapper_node
        rclcpp
        tf2_msgs
)

# install python module for this package
ament_python_install_package(${PROJECT_NAME})

# install applications executables
install(TARGETS ackermann_steering_controller_remapper_node
        DESTINATION lib/${PROJECT_NAME}
)

# install config files, launch scripts
install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        # the following line skips the linter which checks for copyrights
        # comment the line when a copyright and license is added to all source files
        set(ament_cmake_copyright_FOUND TRUE)
        # the following line skips cpplint (only works in a git repo)
        # comment the line when this package is in a git repo and when
        # a copyright and license is added to all source files
        set(ament_cmake_cpplint_FOUND TRUE)
        ament_lint_auto_find_test_dependencies()
endif()

# export package dependencies
ament_export_dependencies(
        ament_cmake
        ament_cmake_python
        rclcpp
        tf2_msgs
)

ament_package()
