cmake_minimum_required(VERSION 3.12)
project(main)
set(CMAKE_CXX_STANDARD 11)

add_executable(main main.cpp)

find_package(OpenCV REQUIRED)

include(cmake/MyBoost.cmake)
include(cmake/MyWs.cmake)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${boost_INCLUDE_DIR}
)

target_link_libraries(main 
    PRIVATE 
    
    ${OpenCV_LIBS}
    video_stream
    server_lib
)

add_subdirectory(src)