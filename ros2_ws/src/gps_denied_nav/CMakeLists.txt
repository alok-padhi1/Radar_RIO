cmake_minimum_required(VERSION 3.8)
project(gps_denied_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(gps_denied_nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

# Frame Converter library
add_library(frame_converter src/frame_converter.cpp)
target_include_directories(frame_converter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(frame_converter
  Eigen3::Eigen
)
ament_target_dependencies(frame_converter
  geometry_msgs
  tf2
)

# Nodes
add_executable(avia_fastlio_adapter src/avia_fastlio_adapter.cpp)
ament_target_dependencies(avia_fastlio_adapter
  rclcpp
  sensor_msgs
)

add_executable(lio_output_adapter src/lio_output_adapter.cpp)
ament_target_dependencies(lio_output_adapter
  rclcpp
  nav_msgs
  gps_denied_nav_msgs
)

add_executable(px4_bridge src/px4_bridge.cpp)
target_link_libraries(px4_bridge frame_converter)
ament_target_dependencies(px4_bridge
  rclcpp
  nav_msgs
  geometry_msgs
)

add_executable(safety_supervisor src/safety_supervisor.cpp)
ament_target_dependencies(safety_supervisor
  rclcpp
  gps_denied_nav_msgs
)

install(TARGETS
  frame_converter
  avia_fastlio_adapter
  lio_output_adapter
  px4_bridge
  safety_supervisor
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

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
  
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frame_converter test/test_frame_converter.cpp)
  target_link_libraries(test_frame_converter frame_converter)
endif()

ament_export_include_directories(include)
ament_export_libraries(frame_converter)
ament_export_dependencies(
  rclcpp
  rclpy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  gps_denied_nav_msgs
)

ament_package()
