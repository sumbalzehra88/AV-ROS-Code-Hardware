cmake_minimum_required(VERSION 3.8)
project(heading_controller_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

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

install(TARGETS
  heading_controller
  gps_distance_velocity_node
  gps_speed_controller
  gps_speed_controller_alert
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
endif()

ament_package()