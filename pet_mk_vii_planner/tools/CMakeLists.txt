
## Plot Bezier Connection Area ##
add_executable(bezier_connection_area)

target_include_directories(bezier_connection_area
  PRIVATE
    ${PROJECT_SOURCE_DIR}/tools/
)

target_sources(bezier_connection_area
  PRIVATE
    ${PROJECT_SOURCE_DIR}/tools/bezier_connection_area.cpp
)

target_link_libraries(bezier_connection_area
    rrt
    ugl::lie_group
    project_options
    project_warnings
    matplot
)

## Velocity Profile ##
add_executable(plot_velocity_profile)

target_include_directories(plot_velocity_profile
  PRIVATE
    ${PROJECT_SOURCE_DIR}/tools/
    ${PROJECT_SOURCE_DIR}/src/include/
)

target_sources(plot_velocity_profile
  PRIVATE
    ${PROJECT_SOURCE_DIR}/tools/plot_velocity_profile.cpp
)

target_link_libraries(plot_velocity_profile
    rrt
    project_options
    project_warnings
    matplot
)


# Ignore warnings coming from matplot.
get_target_property(matplot_include_dirs matplot INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(bezier_connection_area SYSTEM PRIVATE ${matplot_include_dirs})
target_include_directories(plot_velocity_profile SYSTEM PRIVATE ${matplot_include_dirs})

install(
    TARGETS bezier_connection_area
    DESTINATION lib/${PROJECT_NAME}
)

install(
    TARGETS plot_velocity_profile
    DESTINATION lib/${PROJECT_NAME}
)
