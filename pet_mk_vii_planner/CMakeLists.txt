cmake_minimum_required(VERSION 3.8)
project(pet_mk_vii_planner)

find_package(ament_cmake REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(rclcpp REQUIRED)
find_package(ugl REQUIRED)

add_library(project_options INTERFACE)
target_compile_features(project_options INTERFACE cxx_std_17)

add_library(project_warnings INTERFACE)
target_compile_options(project_warnings
  INTERFACE
    -Wall -Wextra -Wpedantic
    -Wshadow
    -Wnon-virtual-dtor
    -Wcast-align
    -Wunused
    -Woverloaded-virtual
    -Wconversion
    # -Wsign-conversion
    -Wnull-dereference
    -Wdouble-promotion
    -Wmisleading-indentation
)

add_subdirectory(src)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  
  add_subdirectory(test)

  include(${PROJECT_SOURCE_DIR}/cmake/matplotplusplus.cmake)
  add_subdirectory(tools)
endif()

ament_package()

add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS=ON)
