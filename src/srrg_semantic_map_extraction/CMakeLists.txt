add_library(srrg_semantic_map_extraction_library SHARED
  lines_extractor.h lines_extractor.cpp
  grid_map.h grid_map.cpp
  semantic_matrix.h semantic_matrix.cpp
  cell.h cell.cpp
  object.h object.cpp
  room_map.h room_map.cpp
  color_extractor.h color_extractor.cpp
  semantic_map_extractor.h semantic_map_extractor.cpp
  structure_analyzer.h structure_analyzer.cpp
)

target_link_libraries(srrg_semantic_map_extraction_library
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
