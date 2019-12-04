cmake_minimum_required(VERSION 3.5)

project(test_ament_cmake NONE)

set(CONAN_DISABLE_CHECK_COMPILER TRUE)
include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

find_package(ament_cmake)

ament_export_dependencies(ament_cmake)

ament_package(
  CONFIG_EXTRAS "ament_cmake_config-extras.cmake"
)
