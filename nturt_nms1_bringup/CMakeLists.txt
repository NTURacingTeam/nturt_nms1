cmake_minimum_required(VERSION 3.8)
project(nturt_nms1_bringup)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_msgs REQUIRED)

include_directories(
        include
)

# libraries
# library: nms1_bringup_remapper
add_library(nms1_bringup_remapper SHARED
        src/nms1_bringup_remapper.cpp
)
ament_target_dependencies(nms1_bringup_remapper
        geometry_msgs
        rclcpp
        rclcpp_components
        tf2_msgs
)
# register to node components
# node component: Nms1BringupRemapper
rclcpp_components_register_nodes(nms1_bringup_remapper "Nms1BringupRemapper")
set(node_plugins "${node_plugins}Nms1BringupRemapper;$<TARGET_FILE:nms1_bringup_remapper>\n")

# executables
# application: nms1_bringup_remapper_node
add_executable(nms1_bringup_remapper_node
        app/nms1_bringup_remapper_node.cpp
)
target_link_libraries(nms1_bringup_remapper_node
        nms1_bringup_remapper
)
ament_target_dependencies(nms1_bringup_remapper_node
        rclcpp
)

# install applications executables
install(TARGETS nms1_bringup_remapper_node
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
        geometry_msgs
        rclcpp
        rclcpp_components
        tf2_msgs
)

ament_package()
