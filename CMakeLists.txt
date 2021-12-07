cmake_minimum_required(VERSION 3.16)

add_compile_options(-W -Wall)

# project info
project(
  routing 
  VERSION 0.1.0
  LANGUAGES CXX
)

# global settings

# activate intellisense
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# include cmake
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})

include(BuildType)
include(GenerateProtobuf)
include(GenerateGRPC)
include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

add_subdirectory(protos)
add_subdirectory(src)
