#include <iostream>
#include <fstream>
#include <string>
#include <thread>

#include "camera.h"
#include "non_maximum_suppression.h"

int main() {

    std::vector<Video_Camera*> list_of_cameras_from_json;
    std::vector<std::pair<std::thread*, non_maximum_suppression*>> list_of_image_processors;
    parse_camera_json(list_of_cameras_from_json);

    for (Video_Camera* c: list_of_cameras_from_json) {
        non_maximum_suppression* img = new non_maximum_suppression(c);
        std::thread* t = new std::thread([&](){
            img->start();
        });
        list_of_image_processors.push_back(std::make_pair(t, img));
    }

    while (true) {
        for (const auto& p: list_of_image_processors) {
            cv::Mat current_frame = p.second->get_current_frame();
            if (current_frame.empty()) continue;
            cv::imshow(p.second->get_url(), current_frame);
            if (cv::waitKey(30) == 27) break;
        }
    }
    
    return 0;
}
