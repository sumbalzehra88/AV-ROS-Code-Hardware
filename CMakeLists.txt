cmake_minimum_required(VERSION 3.8)
project(heading_controller_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

include_directories(include)

# ----------------------------------------------------
# heading_controller executable (source: src/speed.cpp)
# ----------------------------------------------------
add_executable(heading_controller src/speed.cpp)
ament_target_dependencies(heading_controller rclcpp std_msgs)

# ----------------------------------------------------
# gps_distance_velocity_node executable (source: src/speed2.cpp)
# ----------------------------------------------------
add_executable(gps_distance_velocity_node src/speed2.cpp)
ament_target_dependencies(gps_distance_velocity_node rclcpp std_msgs sensor_msgs geometry_msgs)

# ----------------------------------------------------
# gps_speed_controller executable (source: src/speed_controller.cpp)
# ----------------------------------------------------
add_executable(gps_speed_controller src/speed_controller.cpp)
ament_target_dependencies(gps_speed_controller rclcpp std_msgs)

# ----------------------------------------------------
# gps_speed_controller_alert executable (source: src/speed_controller_alert.cpp)
# ----------------------------------------------------
add_executable(gps_speed_controller_alert src/speed_controller_alert.cpp)
ament_target_dependencies(gps_speed_controller_alert rclcpp std_msgs)

# ----------------------------------------------------
# steering_monitor executable (source: src/dynamic.cpp)
# ----------------------------------------------------
add_executable(steering_monitor src/dynamic.cpp)
ament_target_dependencies(steering_monitor rclcpp std_msgs)

# ----------------------------------------------------
# gnss_heading_node executable (source: src/ahrs_sim.cpp)
# ----------------------------------------------------
add_executable(gnss_heading_node src/ahrs_sim.cpp)
ament_target_dependencies(gnss_heading_node rclcpp std_msgs sensor_msgs rcl_interfaces)

# ----------------------------------------------------
# steering_aligner executable (source: src/dynamic_steering_aligner.cpp)
# ----------------------------------------------------
add_executable(steering_aligner src/dynamic_steering_aligner.cpp)
ament_target_dependencies(steering_aligner rclcpp std_msgs)

# ----------------------------------------------------
# gnss_checkpoint_monitor executable (source: src/gnss_checkpoint_monitor.cpp)
# Needs yaml-cpp for YAML checkpoint file loading.
# ----------------------------------------------------
add_executable(gnss_checkpoint_monitor src/gnss_checkpoint_monitor.cpp)
ament_target_dependencies(gnss_checkpoint_monitor rclcpp std_msgs sensor_msgs geometry_msgs)
target_link_libraries(gnss_checkpoint_monitor yaml-cpp)

# ----------------------------------------------------
# stanley_controller executable (source: src/stanley5.cpp)
# ----------------------------------------------------
add_executable(stanley_controller src/stanley5.cpp)
ament_target_dependencies(stanley_controller rclcpp std_msgs nav_msgs ackermann_msgs)

# ----------------------------------------------------
# steering_aligner2 executable (source: src/steering_aligner2.cpp)
# NOTE: this node's internal ROS node name is "steering_aligner", the
# SAME as the steering_aligner target above -- do not run both at once
# without remapping one via --ros-args -r __node:=steering_aligner2.
# ----------------------------------------------------
add_executable(steering_aligner2 src/steering_aligner2.cpp)
ament_target_dependencies(steering_aligner2 rclcpp std_msgs)

install(TARGETS
  heading_controller
  gps_distance_velocity_node
  gps_speed_controller
  gps_speed_controller_alert
  steering_monitor
  gnss_heading_node
  gnss_checkpoint_monitor
  steering_aligner
  stanley_controller
  steering_aligner2
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_heading_controller test/test_speed.cpp)
  target_include_directories(test_heading_controller PUBLIC include)

  ament_add_gtest(test_gps_distance_velocity test/test_speed2.cpp)
  target_include_directories(test_gps_distance_velocity PUBLIC include)

  ament_add_gtest(test_speed_controller test/test_speed_controller.cpp)
  target_include_directories(test_speed_controller PUBLIC include)

  ament_add_gtest(test_speed_controller_alert test/test_speed_controller_alert.cpp)
  target_include_directories(test_speed_controller_alert PUBLIC include)

  ament_add_gtest(test_steering_monitor test/test_dynamic.cpp)
  target_include_directories(test_steering_monitor PUBLIC include)

  ament_add_gtest(test_gnss_heading test/test_ahrs_sim.cpp)
  target_include_directories(test_gnss_heading PUBLIC include)

  ament_add_gtest(test_steering_aligner test/test_dynamic_steering_aligner.cpp)
  target_include_directories(test_steering_aligner PUBLIC include)

  ament_add_gtest(test_checkpoint_monitor test/test_gnss_checkpoint_monitor.cpp)
  target_include_directories(test_checkpoint_monitor PUBLIC include)

  ament_add_gtest(test_stanley5 test/test_stanley5.cpp)
  target_include_directories(test_stanley5 PUBLIC include)

  ament_add_gtest(test_steering_aligner2 test/test_steering_aligner2.cpp)
  target_include_directories(test_steering_aligner2 PUBLIC include)
endif()

ament_package()