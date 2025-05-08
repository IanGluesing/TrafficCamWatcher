# FindWebSocketPP.cmake
# This file uses ExternalProject_Add to download WebSocket++ from GitHub

include(ExternalProject)

ExternalProject_Add(
    websocketpp
    GIT_REPOSITORY https://github.com/zaphoyd/websocketpp.git
    GIT_TAG master
    PREFIX ${CMAKE_BINARY_DIR}/_deps
    GIT_SHALLOW TRUE
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/_install
)

# Include the WebSocket++ headers once downloaded
include_directories(${CMAKE_BINARY_DIR}/_install/include)
