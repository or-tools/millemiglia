PROJECT(mmp_code)

CMAKE_MINIMUM_REQUIRED (VERSION 3.20.0)
SET(CMAKE_CXX_FLAGS "-g -Wall")
file(GLOB_RECURSE sources *.cpp)
file(GLOB_RECURSE headers *.h)
find_package(Protobuf REQUIRED)
add_subdirectory(proto)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
INCLUDE_DIRECTORIES(${Protobuf_INCLUDE_DIRS})

add_library(library ${sources} ${headers})
ADD_EXECUTABLE(main main.cpp)
TARGET_LINK_LIBRARIES(main library proto ${Protobuf_LIBRARIES})
#set_property(TARGET main PROPERTY CXX_STANDARD 20)
