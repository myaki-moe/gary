cmake_minimum_required(VERSION 3.5)
project(gary_common)

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

add_executable(diagnostic_aggregator
        src/diagnostic_aggregator.cpp)

target_include_directories(diagnostic_aggregator PRIVATE include)

ament_target_dependencies(
        diagnostic_aggregator
        rclcpp
        diagnostic_msgs
        lifecycle_msgs
        rclcpp_lifecycle
        rclcpp_components
)

add_library(diagnostic_aggregator_lib SHARED
        src/diagnostic_aggregator.cpp)

target_include_directories(diagnostic_aggregator_lib PRIVATE include)

ament_target_dependencies(
        diagnostic_aggregator_lib
        rclcpp
        diagnostic_msgs
        lifecycle_msgs
        rclcpp_lifecycle
        rclcpp_components
)

rclcpp_components_register_nodes(diagnostic_aggregator_lib "gary_common::DiagnosticAggregator")


add_executable(lifecycle_manager
        src/lifecycle_manager.cpp)

target_include_directories(lifecycle_manager PRIVATE include)

ament_target_dependencies(
        lifecycle_manager
        rclcpp
        diagnostic_msgs
        lifecycle_msgs
        rclcpp_components
)

add_library(lifecycle_manager_lib SHARED
        src/lifecycle_manager.cpp)

target_include_directories(lifecycle_manager_lib PRIVATE include)

ament_target_dependencies(
        lifecycle_manager_lib
        rclcpp
        diagnostic_msgs
        lifecycle_msgs
        rclcpp_components
)

rclcpp_components_register_nodes(lifecycle_manager_lib "gary_common::LifecycleManager")


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


install(TARGETS diagnostic_aggregator lifecycle_manager
        DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS diagnostic_aggregator_lib lifecycle_manager_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

ament_export_dependencies(
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        lifecycle_msgs
        diagnostic_msgs
        )

ament_package()
