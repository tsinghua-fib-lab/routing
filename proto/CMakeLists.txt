find_package(Protobuf REQUIRED)

set(PROTO_SRCS
    simulet/geo/v1/geo.proto
    simulet/map/v1/map.proto
    simulet/map_runtime/v1/map_runtime.proto
    simulet/routing/v1/routing.proto
)
set(PROTO_GEN_OUT_DIR "${PROJECT_SOURCE_DIR}/gen/proto")
file(MAKE_DIRECTORY "${PROTO_GEN_OUT_DIR}")
# use `protobuf_generate` rather than `protobuf_generate_cpp` since we want to
# output the generated files to source dir, rather than binary dir.
protobuf_generate_latest(
    LANGUAGE cpp
    OUT_VAR PROTO_GENERATES
    PROTOC_OUT_DIR "${PROTO_GEN_OUT_DIR}"
    PROTOS ${PROTO_SRCS}
)
message(STATUS "proto generates ${PROTO_GENERATES}")

set_source_files_properties(${PROTO_GENERATES} PROPERTIES GENERATED TRUE)

add_library(generated_proto SHARED ${PROTO_GENERATES})
target_include_directories(generated_proto PUBLIC ${PROTO_GEN_OUT_DIR})
target_link_libraries(generated_proto protobuf::libprotobuf)
