cmake_minimum_required(VERSION 3.8)
project(xtd2_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
)

add_library(coordinate_transform_cpp SHARED
  src/coordinate_transform.cpp
)

target_include_directories(coordinate_transform_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

pybind11_add_module(coordinate_transform_cpp_module
  src/bindings.cpp
)

target_link_libraries(coordinate_transform_cpp_module PRIVATE
  coordinate_transform_cpp
)

set_target_properties(coordinate_transform_cpp_module PROPERTIES
  OUTPUT_NAME coordinate_transform_cpp
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/xtd2_communication
)

ament_python_install_package(${PROJECT_NAME})

# 创建Python可执行文件包装器
install(CODE "
  file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME})
  file(WRITE ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/multirotor_communication
\"#!/usr/bin/env python3
import sys
import os

# 添加Python包路径
script_dir = os.path.dirname(os.path.realpath(__file__))
pkg_dir = os.path.join(script_dir, '..', '..', 'local', 'lib', 'python3.10', 'dist-packages')
sys.path.insert(0, pkg_dir)

# 导入并执行main函数
from xtd2_communication.multirotor_communication import main

if __name__ == '__main__':
    main()
\")
  execute_process(
    COMMAND chmod +x ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/multirotor_communication
  )
")

install(TARGETS coordinate_transform_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS coordinate_transform_cpp_module
  LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  resource
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()
