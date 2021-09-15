add_library(routing_graph
  road_graph.h
  road_graph.cc
)


target_link_libraries(routing_graph
  PUBLIC generated_proto
  PUBLIC tcmalloc
  PRIVATE CONAN_PKG::fmt
  PRIVATE CONAN_PKG::spdlog
)
