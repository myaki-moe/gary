cmake_minimum_required(VERSION 3.5)
project(gary_can)

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
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(socket_can_monitor
        src/socket_can_monitor.cpp)

target_include_directories(socket_can_monitor PRIVATE include)

target_link_libraries(socket_can_monitor socketcan)

ament_target_dependencies(
        socket_can_monitor
        rclcpp
        diagnostic_msgs
        lifecycle_msgs
        rclcpp_lifecycle
        rclcpp_components
)

add_library(socket_can_monitor_lib SHARED
      src/socket_can_monitor.cpp)

target_include_directories(socket_can_monitor_lib PRIVATE include)

target_link_libraries(socket_can_monitor_lib socketcan)

ament_target_dependencies(
        socket_can_monitor_lib
        rclcpp
        diagnostic_msgs
        lifecycle_msgs
        rclcpp_lifecycle
        rclcpp_components
)

rclcpp_components_register_nodes(socket_can_monitor_lib "gary_can::SocketCANMonitor")


add_library(${PROJECT_NAME} SHARED
        src/socket_can_sender.cpp
        src/socket_can_receiver.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC include)

ament_target_dependencies(
        ${PROJECT_NAME}
        rclcpp)


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


# Install
install(TARGETS socket_can_monitor
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS ${PROJECT_NAME} socket_can_monitor_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(
        rclcpp
        diagnostic_msgs
        lifecycle_msgs
        rclcpp_lifecycle
        rclcpp_components
)

ament_package()
