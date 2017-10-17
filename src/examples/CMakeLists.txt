add_executable(grid_map_example grid_map_example.cpp)
target_link_libraries(grid_map_example
  srrg_semantic_map_extraction_library
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${TINYXML_LIBRARIES}
)

add_executable(semantic_map_example semantic_map_example.cpp)
target_link_libraries(semantic_map_example
  srrg_semantic_map_extraction_library
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${TINYXML_LIBRARIES}
)
