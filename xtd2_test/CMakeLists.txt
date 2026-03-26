cmake_minimum_required(VERSION 3.5)
project(xtd2_test)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

SET(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/accel_test.py
  scripts/rosbag_player.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(rosbag_player_cpp src/rosbag_player.cpp)

target_include_directories(rosbag_player_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(rosbag_player_cpp
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  std_msgs
)

install(TARGETS rosbag_player_cpp
  DESTINATION lib/${PROJECT_NAME})

ament_package()
