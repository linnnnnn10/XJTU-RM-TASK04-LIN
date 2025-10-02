cmake_minimum_required(VERSION 3.8)
project(hik_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# 查找MVS SDK
find_path(MVS_INCLUDE_DIRS 
  NAMES MvCameraControl.h 
  PATHS /opt/MVS/include /usr/local/include
)

find_library(MVS_LIBRARIES 
  NAMES MvCameraControl 
  PATHS /opt/MVS/lib/64 /usr/local/lib
)

if(NOT MVS_INCLUDE_DIRS OR NOT MVS_LIBRARIES)
  message(FATAL_ERROR "MVS SDK not found")
else()
  message(STATUS "Found MVS SDK: ${MVS_INCLUDE_DIRS}, ${MVS_LIBRARIES}")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include
  ${MVS_INCLUDE_DIRS}
)

add_executable(hik_camera_node src/hik_camera_node.cpp)
ament_target_dependencies(hik_camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)

target_link_libraries(hik_camera_node ${MVS_LIBRARIES})

install(TARGETS hik_camera_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()