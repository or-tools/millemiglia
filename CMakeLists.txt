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

cmake_minimum_required(VERSION 3.20.0)

project(millemiglia VERSION 0.0.1 LANGUAGES CXX)
set(PROJECT_NAMESPACE millemiglia)
message(STATUS "${PROJECT_NAME} version: ${PROJECT_VERSION}")
#message(STATUS "major: ${PROJECT_VERSION_MAJOR}")
#message(STATUS "minor: ${PROJECT_VERSION_MINOR}")
#message(STATUS "patch: ${PROJECT_VERSION_PATCH}")

include(FetchContent)

message(CHECK_START "Fetching Abseil-cpp")
list(APPEND CMAKE_MESSAGE_INDENT "  ")
set(ABSL_ENABLE_INSTALL ON)
set(ABSL_USE_SYSTEM_INCLUDES ON)
set(ABSL_PROPAGATE_CXX_STD ON)
set(ABSL_BUILD_TESTING OFF)
FetchContent_Declare(
  abseil-cpp
  GIT_REPOSITORY "https://github.com/abseil/abseil-cpp.git"
  GIT_TAG "20240116.1"
  #PATCH_COMMAND git apply --ignore-whitespace "${CMAKE_CURRENT_LIST_DIR}/../../patches/abseil-cpp-20240116.1.patch"
)
FetchContent_MakeAvailable(abseil-cpp)
list(POP_BACK CMAKE_MESSAGE_INDENT)
message(CHECK_PASS "fetched")

message(CHECK_START "Fetching Protobuf")
list(APPEND CMAKE_MESSAGE_INDENT "  ")
set(protobuf_BUILD_TESTS OFF)
#set(protobuf_BUILD_SHARED_LIBS OFF)
set(protobuf_BUILD_EXPORT OFF)
set(protobuf_MSVC_STATIC_RUNTIME OFF)
FetchContent_Declare(
  Protobuf
  GIT_REPOSITORY "https://github.com/protocolbuffers/protobuf.git"
  GIT_TAG "v25.3"
  GIT_SUBMODULES ""
  #PATCH_COMMAND git apply --ignore-whitespace "${CMAKE_CURRENT_LIST_DIR}/../../patches/protobuf-v25.3.patch"
  OVERRIDE_FIND_PACKAGE
)
FetchContent_MakeAvailable(Protobuf)
list(POP_BACK CMAKE_MESSAGE_INDENT)
message(CHECK_PASS "fetched")

message(CHECK_START "Fetching GoogleTest")
list(APPEND CMAKE_MESSAGE_INDENT "  ")
FetchContent_Declare(
  googletest
  # Specify the commit you depend on and update it regularly.
  URL https://github.com/google/googletest/archive/5376968f6948923e2411081fd9372e71a59d8e77.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
list(POP_BACK CMAKE_MESSAGE_INDENT)
message(CHECK_PASS "fetched")

##################
## Proto files  ##
##################
add_subdirectory(proto)

#############
## Library ##
#############
set(CMAKE_CXX_FLAGS "-g -Wall")

file(GLOB_RECURSE sources
  instance_generator/*.cpp
  graph_utils/*.cpp
)
file(GLOB_RECURSE headers
  instance_generator/*.h
  graph_utils/*.h
)

add_library(library ${sources} ${headers})
set_target_properties(library PROPERTIES
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
  POSITION_INDEPENDENT_CODE ON
  INTERFACE_POSITION_INDEPENDENT_CODE ON
  INTERFACE_${PROJECT_NAME}_MAJOR_VERSION ${PROJECT_VERSION_MAJOR}
  COMPATIBLE_INTERFACE_STRING ${PROJECT_NAME}_MAJOR_VERSION
)
target_compile_features(library PUBLIC
  $<IF:$<CXX_COMPILER_ID:MSVC>,cxx_std_20,cxx_std_17>)
target_link_libraries(library PUBLIC
  protobuf::libprotobuf
  millemiglia::millemiglia_proto)

add_executable(main main.cpp)
target_link_libraries(main PRIVATE library)

# Testing
enable_testing()

#Adding tests
file(GLOB_RECURSE TESTS tests/*.cpp)
add_executable(millemiglia_test ${TESTS})
target_link_libraries(millemiglia_test gtest_main library)
add_test(NAME millemiglia_test COMMAND millemiglia_test)
#set_property(TARGET main PROPERTY CXX_STANDARD 20)
