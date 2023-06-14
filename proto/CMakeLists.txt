# Copyright 2023-2024 Google LLC
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# Get Protobuf include dir
set(PROTO_DIRS)
get_target_property(protobuf_dirs protobuf::libprotobuf INTERFACE_INCLUDE_DIRECTORIES)
foreach(dir IN LISTS protobuf_dirs)
  if (NOT "${dir}" MATCHES "INSTALL_INTERFACE|-NOTFOUND")
    #message(STATUS "Adding proto path: ${dir}")
    list(APPEND PROTO_DIRS "--proto_path=${dir}")
  endif()
endforeach()
list(APPEND PROTO_DIRS "--proto_path=${CMAKE_CURRENT_SOURCE_DIR}")

set(PROTOC_PRG protobuf::protoc)

# Generate C++ OBJECT library from proto files,
# e.g
# generate_proto_library(
#   NAME
#     millemiglia_proto
#   FILES
#     millemiglia/foo/foo.proto
#     millemiglia/bar/bar.proto
#  LINK_LIBRARIES
#     millemiglia::millemiglia_proto
#   NO_ALIAS
# )
function(generate_proto_library)
  set(options NO_ALIAS)
  set(oneValueArgs NAME)
  set(multiValueArgs FILES LINK_LIBRARIES)
  cmake_parse_arguments(PROTO
    "${options}"
    "${oneValueArgs}"
    "${multiValueArgs}"
    ${ARGN}
  )

 if(NOT PROTOC_PRG)
   message(FATAL_ERROR "protoc binary not found.")
 endif()

 # Generate proto C++ files.
 set(PROTO_HDRS)
 set(PROTO_SRCS)
 foreach(PROTO_FILE IN LISTS PROTO_FILES)
   message(STATUS "protoc proto(cc): ${PROTO_FILE}")
   get_filename_component(PROTO_NAME_WE ${PROTO_FILE} NAME_WE)
   set(PROTO_HDR ${PROJECT_BINARY_DIR}/proto/${PROTO_NAME_WE}.pb.h)
   set(PROTO_SRC ${PROJECT_BINARY_DIR}/proto/${PROTO_NAME_WE}.pb.cc)
   message(STATUS "protoc hdr: ${PROTO_HDR}")
   message(STATUS "protoc src: ${PROTO_SRC}")
   add_custom_command(
     OUTPUT ${PROTO_SRC} ${PROTO_HDR}
     COMMAND ${PROTOC_PRG}
     "--proto_path=${PROJECT_SOURCE_DIR}"
     ${PROTO_DIRS}
     "--cpp_out=${PROJECT_BINARY_DIR}/proto"
     ${PROTO_FILE}
     DEPENDS ${PROTO_FILE} ${PROTOC_PRG}
     COMMENT "Generate C++ protocol buffer for ${PROTO_FILE}"
     VERBATIM)
   list(APPEND PROTO_HDRS ${PROTO_HDR})
   list(APPEND PROTO_SRCS ${PROTO_SRC})
 endforeach()

 # Create library
 add_library(${PROTO_NAME}_proto OBJECT ${PROTO_SRCS} ${PROTO_HDRS})
 target_compile_features(${PROTO_NAME}_proto PUBLIC $<IF:$<CXX_COMPILER_ID:MSVC>,cxx_std_20,cxx_std_17>)
 if(MSVC)
   set_target_properties(${PROTO_NAME}_proto PROPERTIES CXX_STANDARD 20)
 else()
   set_target_properties(${PROTO_NAME}_proto PROPERTIES CXX_STANDARD 17)
 endif()
 set_target_properties(${PROTO_NAME}_proto PROPERTIES
   CXX_STANDARD_REQUIRED ON
   CXX_EXTENSIONS OFF
   POSITION_INDEPENDENT_CODE ON)
 target_include_directories(${PROTO_NAME}_proto
   PUBLIC
     ${PROJECT_BINARY_DIR}
   PRIVATE
     ${PROJECT_SOURCE_DIR}
   #$<TARGET_PROPERTY:protobuf::libprotobuf,INTERFACE_INCLUDE_DIRECTORIES>
 )
 target_compile_definitions(${PROTO_NAME}_proto PUBLIC ${MILLEMIGLIA_COMPILE_DEFINITIONS})
 target_compile_options(${PROTO_NAME}_proto PUBLIC ${MILLEMIGLIA_COMPILE_OPTIONS})
 target_link_libraries(${PROTO_NAME}_proto PUBLIC protobuf::libprotobuf ${PROTO_LINK_LIBRARIES})
 add_library(${PROJECT_NAMESPACE}::${PROTO_NAME}_proto ALIAS ${PROTO_NAME}_proto)
 message(STATUS "Proto target alias: ${PROJECT_NAMESPACE}::${PROTO_NAME}_proto")
endfunction()

# Generate Protobuf cpp sources
set(MILLEMIGLIA_PROTO_FILES)
file(GLOB MILLEMIGLIA_PROTO_FILES RELATIVE ${PROJECT_SOURCE_DIR}/proto
  "*.proto"
)

generate_proto_library(
  NAME ${PROJECT_NAME}
  FILES ${MILLEMIGLIA_PROTO_FILES})

#INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIR})
#PROTOBUF_GENERATE_CPP(PROTO_SRC PROTO_HEADER resource.proto field_behavior.proto datetime.proto latlng.proto int128.proto millemiglia.proto)
#ADD_LIBRARY(proto ${PROTO_HEADER} ${PROTO_SRC})
