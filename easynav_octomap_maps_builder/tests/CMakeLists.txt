ament_add_gtest(octomap_maps_builder_tests octomap_maps_builder_tests.cpp)
target_include_directories(octomap_maps_builder_tests PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(octomap_maps_builder_tests
  ${PROJECT_NAME}
)
ament_target_dependencies(octomap_maps_builder_tests ${dependencies})


