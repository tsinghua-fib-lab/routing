include_directories(.)

add_executable(routing main.cpp)

target_link_libraries(routing routing_map_loader tcmalloc)
conan_target_link_libraries(routing)

add_subdirectory(map_loader)
