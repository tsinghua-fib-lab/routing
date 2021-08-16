add_library(routing_graph SHARED
    road_graph.h
    road_graph.cc)

conan_target_link_libraries(routing_graph)

target_link_libraries(routing_graph generated_proto tcmalloc)
