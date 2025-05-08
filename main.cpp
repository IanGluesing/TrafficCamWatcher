
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>
#include <opencv2/optflow/motempl.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <iomanip>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>

typedef websocketpp::server<websocketpp::config::asio> server;
typedef websocketpp::connection_hdl connection_hdl;


#include <set>

#include "video_stream.h"

using namespace cv;

std::set<VideoStream*> stream_set = {
    new VideoStream("https://wzmedia.dot.ca.gov/D3/50_24th_St_SAC50_WB.stream/chunklist_w1659799971.m3u8") // https://wzmedia.dot.ca.gov/D3/50_24th_St_SAC50_WB.stream/playlist.m3u8
};

std::set<connection_hdl, std::owner_less<connection_hdl>> clients;

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

int main(int argc, char* argv[]) {
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

    server ws_server;

    ws_server.init_asio();

    ws_server.set_open_handler([&](connection_hdl hdl) {
        std::cout << "Client connected" << std::endl;
        clients.insert(hdl);
    });

    ws_server.set_close_handler([&](connection_hdl hdl) {
        std::cout << "Client disconnected" << std::endl;
        clients.erase(hdl);
    });

    // Optional: handle messages from clients if you want
    ws_server.set_message_handler([](connection_hdl, server::message_ptr msg) {
        std::cout << "Received from browser: " << msg->get_payload() << std::endl;
    });

    ws_server.listen(8765);
    ws_server.start_accept();

    std::thread server_thread([&]() {
        ws_server.run();
    });

    cv::Mat H = cv::findHomography(imagePoints, worldPoints);

    cv::ocl::setUseOpenCL(true);

    // Open video or webcam
    cv::VideoCapture cap("https://wzmedia.dot.ca.gov/D3/50_24th_St_SAC50_WB.stream/chunklist_w1659799971.m3u8");  // Use "video.mp4" for video file
    if (!cap.isOpened()) {
        std::cerr << "Error opening video stream" << std::endl;
        return -1;
    }

    // Read first frame
    cv::Mat frame;
    cap.read(frame);

    // Select ROI for the object you want to track
    cv::Point point1(1220, 230);
    cv::Point point2(150, 480);
    cv::Point point3(1160, 719);
    cv::Point point4(1050, 185);

    // Draw the point (as a small filled circle)
    cv::circle(frame, point1, 2, cv::Scalar(0, 255, 0), -1);
    cv::circle(frame, point2, 2, cv::Scalar(0, 255, 0), -1);
    cv::circle(frame, point3, 2, cv::Scalar(0, 255, 0), -1);
    cv::circle(frame, point4, 2, cv::Scalar(0, 255, 0), -1);

    cv::Rect2d bbox = cv::selectROI("Select Object", frame, false, false);
    cv::destroyWindow("Select Object");
    cv::Rect bbox_int = bbox; 

    // Create a CSRT tracker
    cv::Ptr<cv::Tracker> tracker = cv::TrackerCSRT::create();

    // Initialize tracker with the first frame and bounding box
    tracker->init(frame, bbox_int);

    while (cap.read(frame)) {
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
            ss << std::fixed << std::setprecision(10) << "{\"lat\": " << current_lat << "," << "\"lon\": " << current_long << "}";

            for (auto hdl : clients) {
                ws_server.send(hdl, ss.str(), websocketpp::frame::opcode::text);
            }

            cv::putText(frame, "Tracking", cv::Point(20, 40),
                        cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        } else {
            // Tracking failure
            cv::putText(frame, "Lost Tracking", cv::Point(20, 40),
                        cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        }

        // Display result
        cv::imshow("Tracking", frame);

        // Break loop on ESC key
        if (cv::waitKey(1) == 27) break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;

}
