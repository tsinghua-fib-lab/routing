add_library(routing_map_loader SHARED
    mongo_loader.h
    mongo_loader.cc)

conan_target_link_libraries(routing_map_loader)

target_link_libraries(routing_map_loader generated_proto tcmalloc)
