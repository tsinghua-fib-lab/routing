function(proto_generate_cpp)
  set(_options _)
  set(_singleargs OUT_VAR PROTOC_OUT_DIR)
  if(COMMAND target_sources)
    list(APPEND _singleargs TARGET)
  endif()
  set(_multiargs PROTOS IMPORT_DIRS)

  cmake_parse_arguments(proto_generate_cpp "${_options}" "${_singleargs}" "${_multiargs}" ${ARGN})

  if(NOT proto_generate_cpp_PROTOS AND NOT proto_generate_cpp_TARGET)
    message(SEND_ERROR "Error: proto_generate_cpp called without any targets or source files")
    return()
  endif()

  if(NOT proto_generate_cpp_OUT_VAR AND NOT proto_generate_cpp_TARGET)
    message(SEND_ERROR "Error: proto_generate_cpp called without a target or output variable")
    return()
  endif()

  set(proto_generate_cpp_LANGUAGE cpp)
  set(proto_generate_cpp_GENERATE_EXTENSIONS .pb.h .pb.cc)

  if(NOT proto_generate_cpp_PROTOC_OUT_DIR)
    set(proto_generate_cpp_PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR})
  endif()

  if(proto_generate_cpp_TARGET)
    get_target_property(_source_list ${proto_generate_cpp_TARGET} SOURCES)
    foreach(_file ${_source_list})
      if(_file MATCHES "proto$")
        list(APPEND proto_generate_cpp_PROTOS ${_file})
      endif()
    endforeach()
  endif()

  if(NOT proto_generate_cpp_PROTOS)
    message(SEND_ERROR "Error: proto_generate_cpp could not find any .proto files")
    return()
  endif()

  set(_protobuf_include_path)
  foreach(DIR ${proto_generate_cpp_IMPORT_DIRS})
    get_filename_component(ABS_PATH ${DIR} ABSOLUTE)
    list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
    if(${_contains_already} EQUAL -1)
        list(APPEND _protobuf_include_path -I ${ABS_PATH})
    endif()
  endforeach()

  set(_generated_srcs_all)
  foreach(_proto ${proto_generate_cpp_PROTOS})
    get_filename_component(_abs_file ${_proto} ABSOLUTE)
    get_filename_component(_abs_dir ${_abs_file} DIRECTORY)
    get_filename_component(_basename ${_proto} NAME_WE)
    file(RELATIVE_PATH _rel_dir ${CMAKE_CURRENT_SOURCE_DIR} ${_abs_dir})

    set(_possible_rel_dir)
    if (NOT proto_generate_cpp_APPEND_PATH)
        set(_possible_rel_dir ${_rel_dir}/)
    endif()

    set(_generated_srcs)
    foreach(_ext ${proto_generate_cpp_GENERATE_EXTENSIONS})
      list(APPEND _generated_srcs "${proto_generate_cpp_PROTOC_OUT_DIR}/${_possible_rel_dir}${_basename}${_ext}")
    endforeach()
    list(APPEND _generated_srcs_all ${_generated_srcs})

    add_custom_command(
      OUTPUT ${_generated_srcs}
      COMMAND  protobuf::protoc
      ARGS --${proto_generate_cpp_LANGUAGE}_out ${proto_generate_cpp_PROTOC_OUT_DIR} ${_protobuf_include_path} ${_abs_file}
      DEPENDS ${_abs_file} protobuf::protoc
      COMMENT "Running ${proto_generate_cpp_LANGUAGE} protocol buffer compiler on ${_proto}"
      VERBATIM
    )
  endforeach()

  set_source_files_properties(${_generated_srcs_all} PROPERTIES GENERATED TRUE)
  if(proto_generate_cpp_OUT_VAR)
    set(${proto_generate_cpp_OUT_VAR} ${_generated_srcs_all} PARENT_SCOPE)
  endif()
  if(proto_generate_cpp_TARGET)
    target_sources(${proto_generate_cpp_TARGET} PRIVATE ${_generated_srcs_all})
  endif()
endfunction()
