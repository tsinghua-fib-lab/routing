function(grpc_generate_cpp)
  set(_options _)
  set(_singleargs OUT_VAR PROTOC_OUT_DIR)
  if(COMMAND target_sources)
    list(APPEND _singleargs TARGET)
  endif()
  set(_multiargs PROTOS IMPORT_DIRS)

  cmake_parse_arguments(grpc_generate_cpp "${_options}" "${_singleargs}" "${_multiargs}" ${ARGN})

  if(NOT grpc_generate_cpp_PROTOS AND NOT grpc_generate_cpp_TARGET)
    message(SEND_ERROR "Error: grpc_generate_cpp called without any targets or source files")
    return()
  endif()

  if(NOT grpc_generate_cpp_OUT_VAR AND NOT grpc_generate_cpp_TARGET)
    message(SEND_ERROR "Error: grpc_generate_cpp called without a target or output variable")
    return()
  endif()

  if(NOT grpc_generate_cpp_PROTOC_OUT_DIR)
    set(grpc_generate_cpp_PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR})
  endif()

  set(grpc_generate_cpp_GENERATE_EXTENSIONS .grpc.pb.h .grpc.pb.cc)

  if(grpc_generate_cpp_TARGET)
    get_target_property(_source_list ${grpc_generate_cpp_TARGET} SOURCES)
    foreach(_file ${_source_list})
      if(_file MATCHES "proto$")
        list(APPEND grpc_generate_cpp_PROTOS ${_file})
      endif()
    endforeach()
  endif()

  if(NOT grpc_generate_cpp_PROTOS)
    message(SEND_ERROR "Error: grpc_generate_cpp could not find any .proto files")
    return()
  endif()

  set(_protobuf_include_path)
  foreach(DIR ${grpc_generate_cpp_IMPORT_DIRS})
    get_filename_component(ABS_PATH ${DIR} ABSOLUTE)
    list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
    if(${_contains_already} EQUAL -1)
        list(APPEND _protobuf_include_path -I ${ABS_PATH})
    endif()
  endforeach()

  get_target_property(_grpc_cpp_plugin gRPC::grpc_cpp_plugin IMPORTED_LOCATION) 
  set(_generated_srcs_all)
  foreach(_proto ${grpc_generate_cpp_PROTOS})
    get_filename_component(_abs_file ${_proto} ABSOLUTE)
    get_filename_component(_abs_dir ${_abs_file} DIRECTORY)
    get_filename_component(_basename ${_proto} NAME_WE)
    file(RELATIVE_PATH _rel_dir ${CMAKE_CURRENT_SOURCE_DIR} ${_abs_dir})

    set(_possible_rel_dir ${_rel_dir}/)

    set(_generated_srcs)
    foreach(_ext ${grpc_generate_cpp_GENERATE_EXTENSIONS})
      list(APPEND _generated_srcs "${grpc_generate_cpp_PROTOC_OUT_DIR}/${_possible_rel_dir}${_basename}${_ext}")
    endforeach()
    list(APPEND _generated_srcs_all ${_generated_srcs})
    
    add_custom_command(
      OUTPUT ${_generated_srcs}
      COMMAND  protobuf::protoc
      ARGS --grpc_out ${grpc_generate_cpp_PROTOC_OUT_DIR} --plugin protoc-gen-grpc=${_grpc_cpp_plugin} ${_protobuf_include_path} ${_abs_file}
      DEPENDS ${_abs_file} protobuf::protoc gRPC::grpc_cpp_plugin
      COMMENT "Running c++ grpc protocol buffer compiler on ${_proto}"
      VERBATIM
    )
  endforeach()

  set_source_files_properties(${_generated_srcs_all} PROPERTIES GENERATED TRUE)
  if(grpc_generate_cpp_OUT_VAR)
    set(${grpc_generate_cpp_OUT_VAR} ${_generated_srcs_all} PARENT_SCOPE)
  endif()
  if(grpc_generate_cpp_TARGET)
    target_sources(${grpc_generate_cpp_TARGET} PRIVATE ${_generated_srcs_all})
  endif()
endfunction()
