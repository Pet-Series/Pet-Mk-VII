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

install(
    TARGETS bezier_connection_area
    DESTINATION lib/${PROJECT_NAME}
)

# get_target_property(matplot_includes matplot INCLUDE_DIRECTORIES)
# message("Matplot Includes: "${matplot_includes})
