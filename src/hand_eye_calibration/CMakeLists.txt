cmake_minimum_required(VERSION 3.8)
project(hand_eye_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)

# Find yaml-cpp
pkg_check_modules(YAML_CPP yaml-cpp REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# ============ Shared Library ============
# Calibration utilities library
add_library(calibration_utils SHARED
  src/calibration_utils.cpp
)

ament_target_dependencies(calibration_utils
  Eigen3
  OpenCV
)

# 旧版本（保留注释，暂不启用）:
# add_executable(hand_eye_calibration_node
#   src/hand_eye_calibration_node.cpp
# )
#
# target_link_libraries(hand_eye_calibration_node
#   calibration_utils
#   ${YAML_CPP_LIBRARIES}
# )
#
# ament_target_dependencies(hand_eye_calibration_node
#   rclcpp
#   geometry_msgs
#   sensor_msgs
#   std_srvs
#   cv_bridge
#   OpenCV
#   Eigen3
# )

add_executable(chessboard_tf_publisher_node
  src/chessboard_tf_publisher_node.cpp
)

ament_target_dependencies(chessboard_tf_publisher_node
  rclcpp
  geometry_msgs
  sensor_msgs
  cv_bridge
  tf2_ros
  OpenCV
  Eigen3
)

# ============ Installation ============
install(TARGETS
  # 旧版本（保留注释，暂不安装）
  # hand_eye_calibration_node
  chessboard_tf_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  calibration_utils
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  calib.yaml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ============ Testing ============
#if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  #ament_lint_auto_find_test_files()
#endif()

# ============ Package ============
ament_package()
