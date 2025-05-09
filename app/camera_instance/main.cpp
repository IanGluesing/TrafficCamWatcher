
#include "camera.h"

const double start_lat = 41.693802;
const double start_long = -91.638302;

int main() {
    std::vector<cv::Point2f> imagePoints = {
        cv::Point2f(78, 40),
        cv::Point2f(125, 40),
        cv::Point2f(480, 120),
        cv::Point2f(480, 210),
        cv::Point2f(125, 200)
    };

    std::vector<cv::Point2f> worldPoints = {
        cv::Point2f(0.0, 0.0),
        cv::Point2f(35.0,0),
        cv::Point2f(35.0, 150),
        cv::Point2f(17.5, 160),
        cv::Point2f(0, 150)
    };

    Camera cam(
        "https://iowadotsfs1.us-east-1.skyvdn.com:443/rtplive/ictv03lb/playlist.m3u8",
        start_lat,
        start_long,
        imagePoints,
        worldPoints,
        150,
        5
    );

    cam.start_thread();
    
    return 0;
}
