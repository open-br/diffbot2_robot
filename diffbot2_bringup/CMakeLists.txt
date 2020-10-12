cmake_minimum_required(VERSION 3.5)
project(diffbot2_bringup)

find_package(ament_cmake REQUIRED)

# Auto lint
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
