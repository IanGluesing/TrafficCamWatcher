add_library(camera_lib
    
    camera.cpp
)

target_include_directories(
    camera_lib
        PUBLIC

    ${CMAKE_CURRENT_SOURCE_DIR}/
)

target_link_libraries(camera_lib


    data_forwarder_library
)