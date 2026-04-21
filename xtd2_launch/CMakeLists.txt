cmake_minimum_required(VERSION 3.8)
project(xtd2_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(PCL REQUIRED COMPONENTS common filters)

find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  FetchContent_Declare(
    json
    URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz
  )
  FetchContent_MakeAvailable(json)
endif()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(web_pointcloud_bridge
  src/web_pointcloud_bridge.cpp
)

target_include_directories(web_pointcloud_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(web_pointcloud_bridge
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
  std_msgs
  builtin_interfaces
)

target_link_libraries(web_pointcloud_bridge ${PCL_LIBRARIES})

if(nlohmann_json_FOUND)
  target_link_libraries(web_pointcloud_bridge nlohmann_json::nlohmann_json)
else()
  target_link_libraries(web_pointcloud_bridge nlohmann_json::nlohmann_json)
endif()

install(TARGETS web_pointcloud_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  rviz
  urdf
  web
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

# 创建Python可执行文件包装器
install(CODE "
  file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME})
  
  # tf_publisher
  file(WRITE ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/tf_publisher
\"#!/usr/bin/env python3
import sys
import os

script_dir = os.path.dirname(os.path.realpath(__file__))
pkg_dir = os.path.join(script_dir, '..', '..', 'local', 'lib', 'python3.10', 'dist-packages')
sys.path.insert(0, pkg_dir)

from xtd2_launch.utils.tf_publisher import main

if __name__ == '__main__':
    main()
\")
  execute_process(COMMAND chmod +x ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/tf_publisher)
  
  # tf_publisher_robot_state
  file(WRITE ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/tf_publisher_robot_state
\"#!/usr/bin/env python3
import sys
import os

script_dir = os.path.dirname(os.path.realpath(__file__))
pkg_dir = os.path.join(script_dir, '..', '..', 'local', 'lib', 'python3.10', 'dist-packages')
sys.path.insert(0, pkg_dir)

from xtd2_launch.utils.tf_publisher_robot_state import main

if __name__ == '__main__':
    main()
\")
  execute_process(COMMAND chmod +x ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/tf_publisher_robot_state)
  
  # goal_pose_marker
  file(WRITE ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/goal_pose_marker
\"#!/usr/bin/env python3
import sys
import os

script_dir = os.path.dirname(os.path.realpath(__file__))
pkg_dir = os.path.join(script_dir, '..', '..', 'local', 'lib', 'python3.10', 'dist-packages')
sys.path.insert(0, pkg_dir)

from xtd2_launch.utils.goal_pose_interactive_marker import main

if __name__ == '__main__':
    main()
\")
  execute_process(COMMAND chmod +x ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/goal_pose_marker)
  
  # send_initialpose
  file(WRITE ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/send_initialpose
\"#!/usr/bin/env python3
import sys
import os

script_dir = os.path.dirname(os.path.realpath(__file__))
pkg_dir = os.path.join(script_dir, '..', '..', 'local', 'lib', 'python3.10', 'dist-packages')
sys.path.insert(0, pkg_dir)

from xtd2_launch.utils.send_initialpose import main

if __name__ == '__main__':
    main()
\")
  execute_process(COMMAND chmod +x ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/send_initialpose)
  
  # gazebo_launch
  file(WRITE ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/gazebo_launch
\"#!/usr/bin/env python3
import sys
import os

script_dir = os.path.dirname(os.path.realpath(__file__))
pkg_dir = os.path.join(script_dir, '..', '..', 'local', 'lib', 'python3.10', 'dist-packages')
sys.path.insert(0, pkg_dir)

from xtd2_launch.utils.gazebo_launch import main

if __name__ == '__main__':
    main()
\")
  execute_process(COMMAND chmod +x ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/gazebo_launch)
  
  # px4_launch
  file(WRITE ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/px4_launch
\"#!/usr/bin/env python3
import sys
import os

script_dir = os.path.dirname(os.path.realpath(__file__))
pkg_dir = os.path.join(script_dir, '..', '..', 'local', 'lib', 'python3.10', 'dist-packages')
sys.path.insert(0, pkg_dir)

from xtd2_launch.utils.px4_launch import main

if __name__ == '__main__':
    main()
\")
  execute_process(COMMAND chmod +x ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/px4_launch)
  
  # bridge_launch
  file(WRITE ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/bridge_launch
\"#!/usr/bin/env python3
import sys
import os

script_dir = os.path.dirname(os.path.realpath(__file__))
pkg_dir = os.path.join(script_dir, '..', '..', 'local', 'lib', 'python3.10', 'dist-packages')
sys.path.insert(0, pkg_dir)

from xtd2_launch.utils.bridge_launch import main

if __name__ == '__main__':
    main()
\")
  execute_process(COMMAND chmod +x ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/bridge_launch)
")

ament_package()
