cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(vision_moveit_executor
  src/vision_moveit_executor.cpp
)
ament_target_dependencies(vision_moveit_executor
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  sensor_msgs
  std_msgs
)
install(TARGETS vision_moveit_executor DESTINATION lib/${PROJECT_NAME})

add_executable(view_goal_solver
  src/view_goal_solver.cpp
)
ament_target_dependencies(view_goal_solver
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
install(TARGETS view_goal_solver DESTINATION lib/${PROJECT_NAME})

add_executable(tf_compat_broadcaster
  src/tf_compat_broadcaster.cpp
)
ament_target_dependencies(tf_compat_broadcaster
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)
install(TARGETS tf_compat_broadcaster DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
