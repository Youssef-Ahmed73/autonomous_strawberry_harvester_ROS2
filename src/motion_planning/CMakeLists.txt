cmake_minimum_required(VERSION 3.8)
project(motion_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Find dependencies ─────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_servo REQUIRED)

# ── joystick_servo_node executable ───────────────────────────────────────────
add_executable(joystick_servo_node
  src/joystick_servo_node.cpp
)

target_include_directories(joystick_servo_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(joystick_servo_node PUBLIC cxx_std_17)

ament_target_dependencies(joystick_servo_node
  rclcpp
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  controller_manager_msgs
  control_msgs
  std_msgs
  std_srvs
  moveit_ros_planning_interface
  moveit_servo
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS joystick_servo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ── Tests (optional) ──────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
