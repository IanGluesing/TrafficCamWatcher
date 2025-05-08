include(ExternalProject)

set(ZMQ_INSTALL_DIR ${CMAKE_BINARY_DIR}/_deps/zmq-install)

ExternalProject_Add(zmq
    PREFIX ${CMAKE_BINARY_DIR}/_deps/zmq
    GIT_REPOSITORY https://github.com/zeromq/libzmq.git
    GIT_TAG v4.3.5
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${ZMQ_INSTALL_DIR}
                -DBUILD_SHARED=OFF
                -DBUILD_STATIC=ON
                -DWITH_PERF_TOOL=OFF
                -DWITH_DOCS=OFF
                -DWITH_TLS=OFF
)

# Ensure paths exist
file(MAKE_DIRECTORY ${ZMQ_INSTALL_DIR}/include)
file(MAKE_DIRECTORY ${ZMQ_INSTALL_DIR}/lib)

# Define imported target
add_library(libzmq STATIC IMPORTED)
set_target_properties(libzmq PROPERTIES
    IMPORTED_LOCATION ${ZMQ_INSTALL_DIR}/lib/libzmq.a
    INTERFACE_INCLUDE_DIRECTORIES ${ZMQ_INSTALL_DIR}/include
)

# Ensure our targets depend on ZeroMQ being built first
add_dependencies(libzmq zmq)
