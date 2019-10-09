cmake_minimum_required(VERSION 3.5)

project(test_rcutils)

find_package(rcutils REQUIRED)

add_library(${PROJECT_NAME} test_package.cpp)
ament_target_dependencies(${PROJECT_NAME}
  "rcutils"
)

ament_export_dependencies(rcutils)
