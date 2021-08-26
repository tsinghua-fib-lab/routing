include_directories(.)

add_executable(routing_server server.cc)

target_link_libraries(routing_server
  PRIVATE routing_map_loader
  PRIVATE routing_graph
  PRIVATE routing_controller
  PRIVATE generated_proto
  PRIVATE tcmalloc
  PRIVATE CONAN_PKG::grpc
  PRIVATE CONAN_PKG::fmt
  PRIVATE CONAN_PKG::spdlog
  PRIVATE CONAN_PKG::abseil
)

add_executable(routing_client client.cc)
target_link_libraries(routing_client 
  PRIVATE generated_proto
  PRIVATE tcmalloc
  PRIVATE CONAN_PKG::grpc
  PRIVATE CONAN_PKG::fmt
  PRIVATE CONAN_PKG::spdlog
  PRIVATE CONAN_PKG::abseil
)

add_executable(routing_profile profile.cc)
target_link_libraries(routing_profile
  PRIVATE routing_map_loader
  PRIVATE routing_graph
  PRIVATE generated_proto
  PRIVATE tcmalloc
  PRIVATE profiler
  PRIVATE CONAN_PKG::fmt
  PRIVATE CONAN_PKG::abseil
)

add_executable(lane_access_generator lane_access_generator.cc)
target_link_libraries(lane_access_generator
  PRIVATE generated_proto
  PRIVATE tcmalloc
  PRIVATE CONAN_PKG::abseil
)

add_subdirectory(controller)
add_subdirectory(graph)
add_subdirectory(map_loader)
