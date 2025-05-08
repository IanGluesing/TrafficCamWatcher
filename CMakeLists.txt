cmake_minimum_required(VERSION 3.12)
project(main)
set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)

include(cmake/MyBoost.cmake)
include(cmake/MyWs.cmake)
include(cmake/MyZmq.cmake)
include(cmake/MyCppZmq.cmake)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${boost_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdParty
)

add_subdirectory(data_forwarder)
add_subdirectory(data_handler)
add_subdirectory(src)

add_subdirectory(app)