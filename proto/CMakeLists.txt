INCLUDE(FindProtobuf)
FIND_PACKAGE(Protobuf REQUIRED)

#ADD_SUBDIRECTORY(storage/util)
#ADD_SUBDIRECTORY(google/api)
#ADD_SUBDIRECTORY(google/type)

INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIR})
PROTOBUF_GENERATE_CPP(PROTO_SRC PROTO_HEADER resource.proto field_behavior.proto datetime.proto latlng.proto int128.proto lattle.proto)
ADD_LIBRARY(proto ${PROTO_HEADER} ${PROTO_SRC})
