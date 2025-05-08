set(program_name camera_instance_main)

add_executable(${program_name} main.cpp)

target_link_libraries(${program_name} 
    PRIVATE 
    
    ${OpenCV_LIBS}
    camera_lib
)