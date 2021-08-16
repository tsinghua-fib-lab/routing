cmake_minimum_required(VERSION 3.16)

if(MSVC)
    add_compile_options(/W3 /WX)
else()
    add_compile_options(-W -Wall -Werror)
endif()

# project info
project(
    routing 
    VERSION 0.1
    LANGUAGES CXX)

# global settings

# activate intellisense
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# set runtime output dir
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/build/")

# include cmake
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})

include(BuildType)
include(GenerateProtobuf)
include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

add_subdirectory(proto)
add_subdirectory(src)
