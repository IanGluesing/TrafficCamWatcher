set(program_name camera_server_main)

add_executable(${program_name} main.cpp)

target_link_libraries(${program_name} 
    PRIVATE 
    
    ${OpenCV_LIBS}
    server_lib
)