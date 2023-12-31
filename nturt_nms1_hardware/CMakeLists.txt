cmake_minimum_required(VERSION 3.8)
project(nturt_nms1_hardware)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(JetsonGPIO REQUIRED)

include_directories(
        include
)

# libraries
# library: nturt_nms1_hardware
add_library(${PROJECT_NAME} SHARED
        src/nms1_hardware.cpp
)
target_link_libraries(${PROJECT_NAME}
        JetsonGPIO::JetsonGPIO
)
ament_target_dependencies(${PROJECT_NAME}
        hardware_interface
        rclcpp
        rclcpp_lifecycle
        pluginlib
)
# register to pluginlib
# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(hardware_interface
        nms1_hardware.xml
)

# install include files
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
)

# install library
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
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

# export
ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)
ament_target_dependencies(${PROJECT_NAME}
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
)

ament_package()
