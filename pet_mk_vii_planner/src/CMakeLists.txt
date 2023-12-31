add_library(rrt)

target_sources(rrt
  PRIVATE
    ${PROJECT_SOURCE_DIR}/src/goal.cpp
    ${PROJECT_SOURCE_DIR}/src/graph.cpp
    ${PROJECT_SOURCE_DIR}/src/rrt.cpp
    ${PROJECT_SOURCE_DIR}/src/collision_map.cpp
)

target_include_directories(rrt
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include/
)

target_link_libraries(rrt
  PUBLIC
    ugl::math
    ugl::lie_group
)

add_executable(rrt_simulation)

target_sources(rrt_simulation
  PRIVATE
    ${PROJECT_SOURCE_DIR}/src/main.cpp
)

target_link_libraries(rrt_simulation
    rrt
)

ament_target_dependencies(rrt_simulation 
    geometry_msgs
    visualization_msgs
    rclcpp
)

install(TARGETS
  rrt_simulation
  DESTINATION lib/${PROJECT_NAME})