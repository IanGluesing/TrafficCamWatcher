
add_library(data_forwarder_library

    data_forwarder.cpp
)

target_include_directories(
    data_forwarder_library
        PUBLIC

    ${CMAKE_CURRENT_SOURCE_DIR}/
)

target_link_libraries(
    data_forwarder_library

    libzmq
    libcppzmq
)
