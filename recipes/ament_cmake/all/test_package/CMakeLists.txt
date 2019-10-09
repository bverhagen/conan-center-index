cmake_minimum_required(VERSION 3.5)

project(test_ament_cmake NONE)

find_package(ament_cmake REQUIRED)


ament_export_dependencies(ament_cmake)

ament_package(
  CONFIG_EXTRAS "ament_cmake_config-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
