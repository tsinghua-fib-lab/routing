add_library(routing_controller SHARED
    client.h
    client.cc)

conan_target_link_libraries(routing_controller)

target_link_libraries(routing_controller tcmalloc)
