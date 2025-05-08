include(ExternalProject)

set(CPPZMQ_INSTALL_DIR ${CMAKE_BINARY_DIR}/_deps/cppzmq-install)

ExternalProject_Add(cppzmq
    PREFIX ${CMAKE_BINARY_DIR}/_deps/cppzmq
    GIT_REPOSITORY https://github.com/zeromq/cppzmq.git
    GIT_TAG v4.10.0
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CPPZMQ_INSTALL_DIR}
)

# Ensure paths exist
file(MAKE_DIRECTORY ${CPPZMQ_INSTALL_DIR}/include)
file(MAKE_DIRECTORY ${CPPZMQ_INSTALL_DIR}/lib)

# Define imported target
add_library(libcppzmq INTERFACE)

# Be sure to also link libzmq wherever this is linked

target_include_directories(libcppzmq INTERFACE
    ${CPPZMQ_INSTALL_DIR}/include/
)

# Ensure our targets depend on ZeroMQ being built first
add_dependencies(libcppzmq cppzmq)
