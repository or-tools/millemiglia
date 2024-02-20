file(GLOB_RECURSE sources *.cpp)
file(GLOB_RECURSE headers *.h)
add_library(graph_utils ${sources} ${headers})
