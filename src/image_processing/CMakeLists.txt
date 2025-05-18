add_library(image_processing_lib
    
    tracking_base.cpp
    tracking_implementations/image_processing.cpp
    nms_multi_tracker/nms_multi_tracker.cpp
    non_max_suppression/non_maximum_suppression.cpp
)

target_include_directories(
    image_processing_lib
        PUBLIC

    ${CMAKE_CURRENT_SOURCE_DIR}/
    ${CMAKE_CURRENT_SOURCE_DIR}/tracking_implementations
    ${CMAKE_CURRENT_SOURCE_DIR}/nms_multi_tracker
    ${CMAKE_CURRENT_SOURCE_DIR}/non_max_suppression
)

target_link_libraries(image_processing_lib

    data_handler
    data_forwarder_library
    camera_lib
)