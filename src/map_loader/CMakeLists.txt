add_library(routing_map_loader SHARED
  mongo_loader.h
  mongo_loader.cc
  file_loader.h
  file_loader.cc
)

target_link_libraries(routing_map_loader
  PUBLIC generated_proto
  PUBLIC tcmalloc
  PRIVATE CONAN_PKG::mongo-cxx-driver
  PRIVATE CONAN_PKG::fmt
  PRIVATE CONAN_PKG::spdlog
)
