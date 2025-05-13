#include <iostream>
#include <fstream>
#include <string>
#include <thread>

#include "camera.h"
#include "image_processing.h"

int main() {

    std::vector<Video_Camera*> list_of_cameras_from_json;
    std::vector<std::thread*> list_of_image_processors;
    parse_camera_json(list_of_cameras_from_json);

    for (Video_Camera* c: list_of_cameras_from_json) {
        image_processing* img = new image_processing(c);
        std::thread* t = new std::thread([&](){
            img->start();
        });
        list_of_image_processors.push_back(t);
    }

    while (true) {
        
    }
    
    return 0;
}
