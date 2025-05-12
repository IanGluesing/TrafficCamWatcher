#pragma once

#include <thread>

#include <fstream>
#include <sstream>
#include <iostream>

#include "flatjson.hpp"
#include "base_camera.h"

const std::string CAMERA_JSON_FILE_PATH("cameras.json");

class Video_Camera: public base_camera {
    public:

        Video_Camera(
            const std::string& url,
            double camera_lattitude,
            double camera_longitude,
            const std::vector<cv::Point2f>& imagePoints,
            const std::vector<cv::Point2f>& worldPoints,
            const double relative_y,
            const double relative_x,
            const double rotation);

        ~Video_Camera();

        void start_thread();

    private:

        std::thread camera_thread;
};

void parse_camera_json(std::vector<Video_Camera*>& list_of_cameras_from_json);