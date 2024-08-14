cmake_minimum_required(VERSION 3.5)
project(gary_gimbal)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(gary_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(gimbal_control src/gimbal_control.cpp)

target_include_directories(gimbal_control PRIVATE include)

ament_target_dependencies(
        gimbal_control
        rclcpp
        sensor_msgs
        std_msgs
        controller_manager_msgs
        gary_msgs
        control_msgs)

add_executable(gimbal_enter src/gimbal_enter.cpp)

target_include_directories(gimbal_enter PRIVATE include)

ament_target_dependencies(
        gimbal_enter
        rclcpp
        gary_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
        gimbal_enter
        gimbal_control
        DESTINATION lib/${PROJECT_NAME})

ament_package()
