add_library(server_lib
    
    camera_server.cpp
)

target_include_directories(
    server_lib
        PUBLIC

    ${CMAKE_CURRENT_SOURCE_DIR}/
)