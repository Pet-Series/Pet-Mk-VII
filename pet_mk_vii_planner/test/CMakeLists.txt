find_package(ament_cmake_gtest REQUIRED)
ament_find_gtest()

ament_add_gtest(${PROJECT_NAME}_rrt_test
    ${PROJECT_SOURCE_DIR}/test/velocityProfile_test.cpp
)

# enable_testing()

# add_executable(${PROJECT_NAME}_rrt_test)

target_sources(${PROJECT_NAME}_rrt_test
  PRIVATE
    ${PROJECT_SOURCE_DIR}/test/velocityProfile_test.cpp
)

target_include_directories(${PROJECT_NAME}_rrt_test PUBLIC
  PRIVATE
    ${PROJECT_SOURCE_DIR}/test/
    ${PROJECT_SOURCE_DIR}/src/include/
)

target_link_libraries(${PROJECT_NAME}_rrt_test
    # GTest::gtest_main
    # gtest_main
    rrt
    ugl::lie_group
    project_options
    project_warnings
)

# include(GoogleTest)
# gtest_discover_tests(${PROJECT_NAME}_rrt_test)
