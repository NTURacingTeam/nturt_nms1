cmake_minimum_required(VERSION 3.8)
project(nturt_nms1_nav)

find_package(ament_cmake REQUIRED)

# install config files, launch scripts
install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
