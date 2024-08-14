cmake_minimum_required(VERSION 3.5)
project(gary_shoot)

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
find_package(gary_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(shooter_controller
        src/shooter_controller.cpp)
add_executable(dr16_forwarder
        src/dr16_forwarder.cpp)
add_executable(heat_controller
        src/heat_controller.cpp)

target_include_directories(shooter_controller PRIVATE include)
target_include_directories(dr16_forwarder PRIVATE include)
target_include_directories(heat_controller PRIVATE include)

ament_target_dependencies(
        shooter_controller
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        lifecycle_msgs
        gary_msgs
        diagnostic_msgs
)
ament_target_dependencies(
        dr16_forwarder
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        lifecycle_msgs
        gary_msgs
        diagnostic_msgs
)
ament_target_dependencies(
        heat_controller
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        lifecycle_msgs
        gary_msgs
        diagnostic_msgs
)


add_library(shooter_controller_lib SHARED
        src/shooter_controller.cpp)
add_library(dr16_forwarder_lib SHARED
        src/dr16_forwarder.cpp)
add_library(heat_controller_lib SHARED
        src/heat_controller.cpp)

target_include_directories(shooter_controller_lib PRIVATE include)
target_include_directories(dr16_forwarder_lib PRIVATE include)
target_include_directories(heat_controller_lib PRIVATE include)

ament_target_dependencies(
        shooter_controller_lib
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        lifecycle_msgs
        gary_msgs
        diagnostic_msgs
)
ament_target_dependencies(
        dr16_forwarder_lib
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        lifecycle_msgs
        gary_msgs
        diagnostic_msgs
)
ament_target_dependencies(
        heat_controller_lib
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        lifecycle_msgs
        gary_msgs
        diagnostic_msgs
)

rclcpp_components_register_nodes(shooter_controller_lib "gary_shoot::ShooterController")
rclcpp_components_register_nodes(dr16_forwarder_lib "gary_shoot::DR16Forwarder")
rclcpp_components_register_nodes(heat_controller_lib "gary_shoot::HeatController")

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

install(TARGETS shooter_controller
        DESTINATION lib/${PROJECT_NAME}
        )
install(TARGETS dr16_forwarder
        DESTINATION lib/${PROJECT_NAME}
        )
install(TARGETS heat_controller
        DESTINATION lib/${PROJECT_NAME}
        )

install(TARGETS shooter_controller_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
install(TARGETS dr16_forwarder_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
install(TARGETS heat_controller_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

ament_export_dependencies(
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        lifecycle_msgs
        gary_msgs
        diagnostic_msgs
)

ament_package()
