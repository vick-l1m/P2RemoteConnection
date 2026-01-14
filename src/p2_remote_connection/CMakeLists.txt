cmake_minimum_required(VERSION 3.10)
project(p2_remote_connection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)

include_directories(include)

add_executable(web_teleop_bridge
  src/web_teleop_bridge.cpp
  src/common/ros2_sport_client.cpp
)
ament_target_dependencies(web_teleop_bridge
  rclcpp
  geometry_msgs
  std_msgs
  unitree_api
  unitree_go
)

install(TARGETS web_teleop_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
