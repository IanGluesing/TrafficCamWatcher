cmake_minimum_required(VERSION 3.12)
project(main)
set(CMAKE_CXX_STANDARD 11)

add_executable(main main.cpp)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(main 
    PRIVATE 
    
    ${OpenCV_LIBS}
    video_stream
)

add_subdirectory(src)