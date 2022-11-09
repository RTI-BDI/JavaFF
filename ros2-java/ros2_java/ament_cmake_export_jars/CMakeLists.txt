cmake_minimum_required(VERSION 3.5)

project(ament_cmake_export_jars NONE)

find_package(ament_cmake_core REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(
  CONFIG_EXTRAS "ament_cmake_export_jars-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
