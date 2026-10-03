 cmake_minimum_required(VERSION 3.8)
project(rrt_node)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(odom_tf src/odom_tf.cpp)
ament_target_dependencies(odom_tf
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

add_executable(odom_imu_tf src/odom_imu_tf.cpp)
ament_target_dependencies(odom_imu_tf
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

install(
  TARGETS
    odom_tf
    odom_imu_tf
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    launch
    launch_ros2
    maps
    scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
