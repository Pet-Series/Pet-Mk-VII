add_executable(bezier_connection_area)

target_include_directories(bezier_connection_area
  PRIVATE
    ${PROJECT_SOURCE_DIR}/test/
)

target_sources(bezier_connection_area
  PRIVATE
    ${PROJECT_SOURCE_DIR}/test/bezier_connection_area.cpp
)

target_link_libraries(bezier_connection_area
    rrt
    ugl::lie_group
    project_options
    project_warnings
    matplot
)

# Ignore warnings coming from matplot.
get_target_property(matplot_include_dirs matplot INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(bezier_connection_area SYSTEM PRIVATE ${matplot_include_dirs})


install(
    TARGETS bezier_connection_area
    DESTINATION lib/${PROJECT_NAME}
)
