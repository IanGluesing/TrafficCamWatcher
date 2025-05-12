#pragma once

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

class base_camera {
    public:

        base_camera(
            const std::string& url,
            double camera_lattitude,
            double camera_longitude,
            const std::vector<cv::Point2f>& imagePoints,
            const std::vector<cv::Point2f>& worldPoints,
            double relative_y,
            double relative_x,
            double rotation) {

            this->url = url;
            this->camera_lattitude = camera_lattitude;
            this->camera_longitude = camera_longitude;
            this->imagePoints = imagePoints;
            this->worldPoints = worldPoints;
            this->relative_y = relative_y;
            this->relative_x = relative_x;
            this->rotation = rotation;
        }

        virtual ~base_camera() = default;

        std::string get_url() const {
            return this->url;
        }

        double get_camera_lattitude() const {
            return camera_lattitude;
        }

        double get_camera_longitude() const {
            return camera_longitude;
        }

        double get_camera_relative_y() const {
            return relative_y;
        }

        double get_camera_relative_x() const {
            return relative_x;
        }

        double get_camera_rotation() const {
            return rotation;
        }

        std::vector<cv::Point2f> get_camera_image_points() const {
            return imagePoints;
        }

        std::vector<cv::Point2f> get_camera_world_points() const {
            return worldPoints;
        }

    protected:

        std::string url;
        double camera_lattitude;
        double camera_longitude;
        double relative_y;
        double relative_x;
        double rotation;

        std::vector<cv::Point2f> imagePoints;
        std::vector<cv::Point2f> worldPoints;
};