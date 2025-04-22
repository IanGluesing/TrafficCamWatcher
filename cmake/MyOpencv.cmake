include(ExternalProject)

include(ExternalProject)

ExternalProject_Add(
    zlib
    URL https://zlib.net/zlib-1.3.1.tar.gz
    PREFIX ${CMAKE_BINARY_DIR}/_deps/zlib
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    INSTALL_DIR ${CMAKE_BINARY_DIR}/_install/zlib
)

# === 2. Set Zlib Paths for OpenCV ===
ExternalProject_Get_Property(zlib install_dir)
set(ZLIB_ROOT ${install_dir})
set(ZLIB_INCLUDE_DIR ${ZLIB_ROOT}/include)
set(ZLIB_LIBRARY ${ZLIB_ROOT}/lib/libz.a)  # Static library

ExternalProject_Add(
    opencv
    GIT_REPOSITORY https://github.com/opencv/opencv.git
    GIT_TAG 4.5.5
    DEPENDS zlib
    PREFIX ${CMAKE_BINARY_DIR}/_deps/opencv
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
        -DCMAKE_PREFIX_PATH=${ZLIB_ROOT}
        -DWITH_ZLIB=ON
        -DZLIB_INCLUDE_DIR=${ZLIB_INCLUDE_DIR}
        -DZLIB_LIBRARY=${ZLIB_LIBRARY}
        -DBUILD_TESTS=OFF
        -DBUILD_PERF_TESTS=OFF
        -DBUILD_EXAMPLES=OFF
        -DBUILD_SHARED_LIBS=ON
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    INSTALL_DIR ${CMAKE_BINARY_DIR}/_install/opencv
)


# Location of installed OpenCV
set(OpenCV_INSTALL_DIR ${CMAKE_BINARY_DIR}/_deps/opencv)

# Add include and lib directories after build
ExternalProject_Get_Property(opencv install_dir)

add_library(opencv_external INTERFACE)
add_dependencies(opencv_external opencv)
target_include_directories(opencv_external INTERFACE ${OpenCV_INSTALL_DIR}/include)
link_directories(${OpenCV_INSTALL_DIR}/lib)

# # Example target that uses OpenCV
# add_executable(my_app main.cpp)
# add_dependencies(my_app opencv)
# target_link_libraries(my_app PRIVATE opencv_external ${OpenCV_INSTALL_DIR}/lib/libopencv_core.so)  # Add more libs as needed
