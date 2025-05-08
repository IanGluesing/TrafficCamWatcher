
#include "camera.h"

const double start_lat = 38.560719;
const double start_long = -121.480659;

int main() {
    std::vector<cv::Point2f> imagePoints = {
        cv::Point2f(150, 480),
        cv::Point2f(1050, 185),
        cv::Point2f(1220, 230),
        cv::Point2f(1160, 719)
    };

    std::vector<cv::Point2f> worldPoints = {
        cv::Point2f(0.0, 0.0),
        cv::Point2f(100.0,0),
        cv::Point2f(80.0, 30.0),
        cv::Point2f(0.0, 30.0)
    };

    Camera cam(
        "https://wzmedia.dot.ca.gov/D3/50_24th_St_SAC50_WB.stream/playlist.m3u8",
        start_lat,
        start_long,
        imagePoints,
        worldPoints,
        30,
        10
    );

    cam.start_thread();
    
    return 0;
}
