add_library(rrt)

target_sources(rrt
  PRIVATE
    ${PROJECT_SOURCE_DIR}/src/bezier.cpp
    ${PROJECT_SOURCE_DIR}/src/collision_map.cpp
    ${PROJECT_SOURCE_DIR}/src/goal.cpp
    ${PROJECT_SOURCE_DIR}/src/graph.cpp
    ${PROJECT_SOURCE_DIR}/src/rrt.cpp
    ${PROJECT_SOURCE_DIR}/src/rrtDefinitions.cpp
    ${PROJECT_SOURCE_DIR}/src/steerBezier.cpp
    ${PROJECT_SOURCE_DIR}/src/steerCtrv.cpp
    ${PROJECT_SOURCE_DIR}/src/utility/algorithm.cpp
    ${PROJECT_SOURCE_DIR}/src/utility/interpolation.cpp
)

target_include_directories(rrt
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include/
  PRIVATE
    ${PROJECT_SOURCE_DIR}/src/include/
)

target_link_libraries(rrt
  PUBLIC
    ugl::math
    ugl::lie_group
  PRIVATE
    ugl::random
    project_options
    project_warnings
)

add_executable(rrt_simulation)

target_include_directories(rrt_simulation
  PRIVATE
    ${PROJECT_SOURCE_DIR}/src/include/
)

target_sources(rrt_simulation
  PRIVATE
    ${PROJECT_SOURCE_DIR}/src/main.cpp
    ${PROJECT_SOURCE_DIR}/src/visualization.cpp
)

target_link_libraries(rrt_simulation
    rrt
    project_options
    project_warnings
)

ament_target_dependencies(rrt_simulation 
    geometry_msgs
    visualization_msgs
    rclcpp
)

install(TARGETS
  rrt_simulation
  DESTINATION lib/${PROJECT_NAME})