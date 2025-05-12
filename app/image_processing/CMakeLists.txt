set(program_name image_processing_main)

add_executable(${program_name} main.cpp)

target_link_libraries(${program_name} 
    PRIVATE 
    
    ${OpenCV_LIBS}
    image_processing_lib
)