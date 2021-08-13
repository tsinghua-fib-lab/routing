# include(FindProtobuf)
set(protobuf_MODULE_COMPATIBLE TRUE)
find_package(Protobuf REQUIRED)
if(Protobuf_FOUND)
  message(STATUS "Using Protocol Buffers ${Protobuf_VERSION}")
  message(STATUS "protobuf include dirs ${Protobuf_INCLUDE_DIRS}")
  message(STATUS "protobuf src root folder ${Protobuf_SRC_ROOT_FOLDER}")
endif()

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
compute_generated_srcs(PROTO_GENERATES_SRCS "${PROTO_GEN_OUT_DIR}" false ${PROTO_SRCS})
message(STATUS "proto generates srcs ${PROTO_GENERATES_SRCS}")
message(STATUS "proto generates ${PROTO_GENERATES}")

# populate `PROTOBUF_GENERATES` in the parent scope.
set(PROTOBUF_GENERATE_DEPS)
foreach(cxx_file ${PROTO_GENERATES})
    if(cxx_file MATCHES "cc$")
        list(APPEND PROTOBUF_GENERATE_DEPS ${cxx_file})
    endif()
endforeach()

set(PROTOBUF_GENERATES ${PROTO_GENERATES_SRCS})
set(PROTOBUF_GENERATES ${PROTOBUF_GENERATES} PARENT_SCOPE)
set_source_files_properties(${PROTOBUF_GENERATES} PROPERTIES GENERATED TRUE)
add_custom_target(protobuf_generates DEPENDS ${PROTOBUF_GENERATE_DEPS})
