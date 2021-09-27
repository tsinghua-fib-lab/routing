add_library(routing_graph
  lane_graph.h
  lane_graph.cc
)


target_link_libraries(routing_graph
  PUBLIC generated_proto
  PUBLIC tcmalloc
  PRIVATE CONAN_PKG::fmt
  PRIVATE CONAN_PKG::spdlog
)
