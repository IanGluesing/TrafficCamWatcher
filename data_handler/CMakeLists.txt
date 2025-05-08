add_library(data_handler

    data_handler.cpp
)

target_include_directories(
    data_handler
        PUBLIC

    ${CMAKE_CURRENT_SOURCE_DIR}/
)

target_link_libraries(
    data_handler

    data_forwarder_library
    libflatjson
)