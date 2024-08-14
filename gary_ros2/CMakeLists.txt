cmake_minimum_required(VERSION 3.5)
project(gary_ros2)

if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

find_package(gary_bringup REQUIRED)
find_package(gary_can REQUIRED)
find_package(gary_common REQUIRED)
find_package(gary_controller REQUIRED)
find_package(gary_description REQUIRED)
find_package(gary_hardware REQUIRED)
find_package(gary_msgs REQUIRED)
find_package(gary_serial REQUIRED)

ament_package()
