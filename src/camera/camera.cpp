#include "camera.h"
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include "data_forwarder.h"

const double earthRadius = 6378137.0; // in meters (WGS84)
const double start_lat = 38.560719;
const double start_long = -121.480659;

double current_lat = start_lat;
double current_long = start_long;

void rotate_point(double& x, double& y, float theta_degrees) {
    // Convert the angle from degrees to radians
    float theta_radians = theta_degrees * M_PI / 180.0;

    // Apply the rotation matrix
    float new_x = x * std::cos(theta_radians) - y * std::sin(theta_radians);
    float new_y = x * std::sin(theta_radians) + y * std::cos(theta_radians);

    // Update the original point with the new rotated coordinates
    x = new_x;
    y = new_y;
}

Camera::Camera(
    const std::string& url,
    double camera_lattitude,
    double camera_longitude,
    const std::vector<cv::Point2f>& imagePoints,
    const std::vector<cv::Point2f>& worldPoints,
    const double relative_y,
    const double relative_x) : 
    camera_lattitude(camera_lattitude), 
    camera_longitude(camera_longitude),
    relative_y(relative_y),
    relative_x(relative_x)
{
    this->url = url;
    this->imagePoints = imagePoints;
    this->worldPoints = worldPoints;
}

Camera::~Camera() {
    if (camera_thread.joinable()) {
        camera_thread.join();
    }
}

void Camera::start_thread() {

    // this->camera_thread = std::thread([&](){ 
        data_forwarder df = data_forwarder("ipc:///tmp/to_camera_server", zmq::socket_type::dealer);
        df.connect();

        cv::Mat H = cv::findHomography(imagePoints, worldPoints);

        cv::ocl::setUseOpenCL(true);
        cv::VideoCapture cap("https://wzmedia.dot.ca.gov/D3/50_24th_St_SAC50_WB.stream/playlist.m3u8");  // Use "video.mp4" for video file
        if (!cap.isOpened()) {
            std::cerr << "Error opening video stream" << std::endl;
            return;
        }

        // Read first frame
        cv::Mat frame;
        cap.read(frame);

        cv::Rect2d bbox = cv::selectROI("Select Object", frame, false, false);
        cv::destroyWindow("Select Object");
        cv::Rect bbox_int = bbox; 
        double fps = cap.get(cv::CAP_PROP_FPS);
        int frame_time_ms = static_cast<int>(1000.0 / fps);

        // Create a CSRT tracker
        cv::Ptr<cv::Tracker> tracker = cv::TrackerCSRT::create();

        // Initialize tracker with the first frame and bounding box
        tracker->init(frame, bbox_int);

        while (cap.read(frame)) {
            auto start = std::chrono::high_resolution_clock::now();
            // Update tracking result
            bool success = tracker->update(frame, bbox_int);

            if (success) {
                // Tracking success — draw rectangle
                cv::rectangle(frame, bbox_int, cv::Scalar(0, 255, 0), 2, 1);

                cv::Point2f center;
                center.x = bbox_int.x + bbox_int.width / 2.0;
                center.y = bbox_int.y + bbox_int.height / 2.0;
                std::vector<cv::Point2f> objectInImage = { center };
                std::vector<cv::Point2f> objectInWorld;

                cv::perspectiveTransform(objectInImage, objectInWorld, H);

                std::cout << "Real-world position: " 
                        << objectInWorld[0].x + 10 << ", " 
                        << 30 - objectInWorld[0].y << std::endl;

                double deltaLat = ((30 - objectInWorld[0].y) / earthRadius) * (180.0 / CV_PI);
                double deltaLon = ((objectInWorld[0].x + 10) / (earthRadius * cos(current_lat * CV_PI / 180.0))) * (180.0 / CV_PI);

                rotate_point(deltaLon, deltaLat, 170);
                current_lat = start_lat + deltaLat;
                current_long = start_long + deltaLon;

                std::cout << "Real-world position: " 
                        << std::fixed << std::setprecision(15)
                        << current_lat << ", " 
                        << current_long << std::endl;

                std::ostringstream ss;
                ss << std::fixed << std::setprecision(10) << "{\"lat\": \"" << current_lat << "\", " << "\"lon\": \"" << current_long << "\"}";
                std::cout<<"sending: "<<ss.str()<<std::endl;
                df.send_message(ss.str().c_str(), ss.str().size());

                cv::putText(frame, "Tracking", cv::Point(20, 40),
                            cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
            } else {
                // Tracking failure
                cv::putText(frame, "Lost Tracking", cv::Point(20, 40),
                            cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
            }

            auto end = std::chrono::high_resolution_clock::now();
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

            if (elapsed_ms < frame_time_ms) {
                std::this_thread::sleep_for(std::chrono::milliseconds(frame_time_ms - elapsed_ms));
            }

        }

        return;
    // });
}