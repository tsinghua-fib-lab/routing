add_executable(routing main.cpp)
target_include_directories(routing PRIVATE
    "${PROJECT_SOURCE_DIR}/gen/proto"
)
target_link_libraries(routing ${CONAN_LIBS})
add_dependencies(routing protobuf_generates)
