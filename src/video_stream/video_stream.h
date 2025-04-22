#pragma once

#include "camera.h"
#include <opencv2/opencv.hpp>

class VideoStream {
    public:

        VideoStream(std::string url) : cam(url) {
            this->cap = cv::VideoCapture(url);
            this->current_frame = cv::Mat();
        }

        void start();

        cv::Mat get_current_frame() const {
            return current_frame.clone();
        }

        std::string get_url() {
            return cam.get_url();
        }

    private:

        Camera cam;

        cv::VideoCapture cap;
        cv::Mat current_frame;
};