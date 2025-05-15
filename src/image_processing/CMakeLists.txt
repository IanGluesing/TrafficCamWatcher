add_library(image_processing_lib
    
    tracking_base.cpp
    tracking_implementations/image_processing.cpp
)

target_include_directories(
    image_processing_lib
        PUBLIC

    ${CMAKE_CURRENT_SOURCE_DIR}/
    ${CMAKE_CURRENT_SOURCE_DIR}/tracking_implementations
)

target_link_libraries(image_processing_lib

    data_handler
    data_forwarder_library
    camera_lib
)