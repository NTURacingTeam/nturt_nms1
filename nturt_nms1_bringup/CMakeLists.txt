cmake_minimum_required(VERSION 3.8)
project(nturt_nms1_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# install python module for this package
ament_python_install_package(${PROJECT_NAME})

# install config files, launch scripts, map files
install(DIRECTORY config launch maps
        DESTINATION share/${PROJECT_NAME}
)

# install python scripts
install(DIRECTORY scripts/
        DESTINATION lib/${PROJECT_NAME}
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
        rclpy
)

ament_package()
