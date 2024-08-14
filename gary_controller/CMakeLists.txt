cmake_minimum_required(VERSION 3.5)
project(gary_controller)

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
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(gary_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(${PROJECT_NAME} SHARED
        src/offline_broadcaster.cpp
        src/pid_controller.cpp
        )

target_include_directories(${PROJECT_NAME} PRIVATE include)

ament_target_dependencies(
        ${PROJECT_NAME}
        controller_interface
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        realtime_tools
        diagnostic_msgs
        gary_msgs
        std_msgs
)

pluginlib_export_plugin_description_file(controller_interface gary_controller.xml)

install(
        DIRECTORY include/
        DESTINATION include
)
install(
        TARGETS
        ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
)


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

ament_export_include_directories(
        include
)

ament_export_dependencies(
        controller_interface
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        realtime_tools
        diagnostic_msgs
        gary_msgs
        std_msgs
)
ament_export_libraries(
        ${PROJECT_NAME}
)

ament_package()
