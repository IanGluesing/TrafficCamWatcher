#pragma once
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <thread>

class Camera {
    public:

        Camera(
            const std::string& url,
            double camera_lattitude,
            double camera_longitude,
            const std::vector<cv::Point2f>& imagePoints,
            const std::vector<cv::Point2f>& worldPoints,
            const double relative_y,
            const double relative_x);

        ~Camera();

        void start_thread();

    private:

        std::thread camera_thread;

        std::string url;
        const double camera_lattitude;
        const double camera_longitude;
        const double relative_y;
        const double relative_x;

        std::vector<cv::Point2f> imagePoints;
        std::vector<cv::Point2f> worldPoints;
};