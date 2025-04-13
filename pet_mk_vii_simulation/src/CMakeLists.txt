add_executable(ackermann_controller)

target_sources(ackermann_controller
  PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/ackermann_controller.cpp
)

target_include_directories(ackermann_controller
  PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(ackermann_controller
    project_options
)

ament_target_dependencies(ackermann_controller
    std_msgs
    rclcpp
)

install(TARGETS ackermann_controller
    DESTINATION lib/${PROJECT_NAME}
)