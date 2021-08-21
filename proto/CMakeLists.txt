include(grpc_cpp_plugin)
set(PROTO_SRCS
    simulet/geo/v1/geo.proto
    simulet/map/v1/map.proto
    simulet/map_runtime/v1/map_runtime.proto
    simulet/route/v1/route.proto
    simulet/route/v1/route_api.proto
)
set(PROTO_GEN_OUT_DIR "${PROJECT_SOURCE_DIR}/gen/proto")
set(PROTO_IMPORT_DIR "${PROJECT_SOURCE_DIR}/proto")
file(MAKE_DIRECTORY "${PROTO_GEN_OUT_DIR}")
proto_generate_cpp(
    OUT_VAR PROTO_GENERATES
    PROTOC_OUT_DIR ${PROTO_GEN_OUT_DIR}
    PROTOS ${PROTO_SRCS}
    IMPORT_DIRS ${PROTO_IMPORT_DIR}
)
message(STATUS "proto generates ${PROTO_GENERATES}")
set(GRPC_SRCS
    simulet/route/v1/route_api.proto
)
message(STATUS "${PROTO_GEN_OUT_DIR} ${GRPC_SRCS} ${PROTO_IMPORT_DIR}")
grpc_generate_cpp(
    OUT_VAR GRPC_GENERATES
    PROTOC_OUT_DIR ${PROTO_GEN_OUT_DIR}
    PROTOS ${GRPC_SRCS}
    IMPORT_DIRS ${PROTO_IMPORT_DIR}
)
message(STATUS "grpc generates ${GRPC_GENERATES}")

add_library(generated_proto SHARED ${PROTO_GENERATES} ${GRPC_GENERATES})
target_include_directories(generated_proto PUBLIC ${PROTO_GEN_OUT_DIR})
conan_target_link_libraries(generated_proto)
