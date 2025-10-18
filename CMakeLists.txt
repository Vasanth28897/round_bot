cmake_minimum_required(VERSION 3.8)
project(round_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-math6 REQUIRED)
# find_package(costmap_converter_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


add_library(move_model SHARED plugins/move_model.cpp)
target_link_libraries(move_model PUBLIC ignition-gazebo6::ignition-gazebo6)
target_include_directories(move_model PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${ignition-gazebo6_INCLUDE_DIRS}
)

install(TARGETS move_model
  LIBRARY DESTINATION lib
)

add_executable(scan_filter_node src/scan_filter_node.cpp)
ament_target_dependencies(scan_filter_node 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  tf2_ros 
  tf2_geometry_msgs
  visualization_msgs
)

install(
  TARGETS scan_filter_node 
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY 
    launch 
    description 
    rviz 
    config
    worlds
    plugins
    src
  DESTINATION 
    share/${PROJECT_NAME}
)

install(
  PROGRAMS
  src/obstacle_processor.py
  src/pose_frame_id_adder.py 
  src/logged_waypoint_follower.py
  src/interactive_waypoint_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

