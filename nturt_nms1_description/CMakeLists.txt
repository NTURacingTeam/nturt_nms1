cmake_minimum_required(VERSION 3.8)
project(nturt_nms1_description)

find_package(ament_cmake REQUIRED)

# install config files, launch scripts, model urdf
install(DIRECTORY config launch urdf
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
