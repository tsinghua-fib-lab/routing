include_directories(.)

add_executable(routing_server server.cc)

target_link_libraries(routing_server
    routing_map_loader
    routing_graph
    generated_proto
    tcmalloc
)
conan_target_link_libraries(routing_server)

add_executable(routing_client client.cc)
target_link_libraries(routing_client 
    generated_proto
    tcmalloc
)
conan_target_link_libraries(routing_client)

add_executable(routing_profile main.cc)
target_link_libraries(routing_profile
    routing_map_loader
    routing_graph
    generated_proto
    tcmalloc
    profiler
)
conan_target_link_libraries(routing_profile)

add_subdirectory(map_loader)
add_subdirectory(graph)
