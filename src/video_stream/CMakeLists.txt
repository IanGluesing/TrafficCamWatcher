add_library(video_stream
    
    camera.cpp
    video_stream.cpp
)

target_include_directories(
    video_stream
        PUBLIC

    ${CMAKE_CURRENT_SOURCE_DIR}/
)