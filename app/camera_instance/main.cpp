
#include "camera.h"

const double start_lat = 41.693802;
const double start_long = -91.638302;

int main() {
    std::vector<cv::Point2f> imagePoints = {
        cv::Point2f(0, 125),
        cv::Point2f(265, 55),
        cv::Point2f(355, 52),
        cv::Point2f(300, 270),
        cv::Point2f(0, 270)
    };

    std::vector<cv::Point2f> worldPoints = {
        cv::Point2f(0.0, 0.0),
        cv::Point2f(150.0,0),
        cv::Point2f(180, 35),
        cv::Point2f(-10, 35),
        cv::Point2f(-15, 20)
    };

    Camera cam(
        "https://iowadotsfs1.us-east-1.skyvdn.com:443/rtplive/ictv03lb/playlist.m3u8",
        start_lat,
        start_long,
        imagePoints,
        worldPoints,
        42,
        8
    );

    cam.start_thread();
    
    return 0;
}
