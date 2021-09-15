add_library(routing_controller
  client.h
  client.cc
)

target_link_libraries(routing_controller
  PUBLIC tcmalloc
  PUBLIC CONAN_PKG::etcd-cpp-api
)
