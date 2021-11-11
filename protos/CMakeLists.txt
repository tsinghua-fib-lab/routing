include(grpc_cpp_plugin)
set(PROTO_SRCS
  wolong/geo/v1/geo.proto
  wolong/map/v1/map.proto
  wolong/trip/v1/trip.proto
  wolong/agent/v1/agent.proto
  wolong/routing/v1/routing.proto
  wolong/routing/v1/routing_api.proto
)
set(PROTO_GEN_OUT_DIR "${PROJECT_BINARY_DIR}/gen/protos")
set(PROTO_IMPORT_DIR "${PROJECT_SOURCE_DIR}/protos")
file(MAKE_DIRECTORY "${PROTO_GEN_OUT_DIR}")
proto_generate_cpp(
  OUT_VAR PROTO_GENERATES
  PROTOC_OUT_DIR ${PROTO_GEN_OUT_DIR}
  PROTOS ${PROTO_SRCS}
  IMPORT_DIRS ${PROTO_IMPORT_DIR}
)
message(STATUS "proto generates ${PROTO_GENERATES}")

set(GRPC_SRCS
  wolong/routing/v1/routing_api.proto
)
grpc_generate_cpp(
  OUT_VAR GRPC_GENERATES
  PROTOC_OUT_DIR ${PROTO_GEN_OUT_DIR}
  PROTOS ${GRPC_SRCS}
  IMPORT_DIRS ${PROTO_IMPORT_DIR}
)
message(STATUS "grpc generates ${GRPC_GENERATES}")

add_library(generated_proto ${PROTO_GENERATES} ${GRPC_GENERATES})
target_include_directories(generated_proto
  PUBLIC ${PROTO_GEN_OUT_DIR}
)
target_link_libraries(generated_proto
  PUBLIC CONAN_PKG::protobuf
  PUBLIC CONAN_PKG::grpc
)
