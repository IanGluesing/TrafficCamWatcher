#include "camera.h"
#include <opencv2/tracking.hpp>

#include <opencv2/core/ocl.hpp>
#include "data_forwarder.h"
#include <opencv2/tracking/tracking_legacy.hpp>

 #include <opencv2/core/utility.hpp>
 #include <opencv2/tracking.hpp>
 #include <opencv2/videoio.hpp>
 #include <opencv2/highgui.hpp>

const double earthRadius = 6378137.0; // in meters (WGS84)

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

bool areRectsSimilar(const cv::Rect2d& r1, const cv::Rect2d& r2, double tol = 30) {
    return (std::abs(r1.x - r2.x) < tol &&
            std::abs(r1.y - r2.y) < tol &&
            std::abs(r1.width - r2.width) < tol &&
            std::abs(r1.height - r2.height) < tol);
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
        cv::VideoCapture cap(url); 
        if (!cap.isOpened()) {
            std::cerr << "Error opening video stream" << std::endl;
            return;
        }


        double fps = cap.get(cv::CAP_PROP_FPS);
        int frame_time_ms = static_cast<int>(1000.0 / fps);


        cv::Ptr<cv::BackgroundSubtractor> bgSubtractor = cv::createBackgroundSubtractorMOG2();
    cv::Mat frame, fgMask;
    cap >> frame;


        // for(const auto& image_point: imagePoints) {
        //     cv::circle(frame, image_point, 2, cv::Scalar(0, 255, 0), -1);
        // }

        // cv::Rect2d bbox = cv::selectROI("Select Object", frame, false, false);
        // cv::destroyWindow("Select Object");
        // cv::Rect bbox_int = bbox; 

    cv::Ptr<cv::legacy::MultiTracker> multiTracker = cv::legacy::MultiTracker::create();

    bool initialized = false;
    int count = 0;
    while (true) {
        auto start = std::chrono::high_resolution_clock::now();

        cap >> frame;
        if (frame.empty()) break;

        if (!initialized || ((count+1) % 5 == 0)) {
            if (((count+1) % 10) == 0) {
                multiTracker = cv::legacy::MultiTracker::create();
            }
            bgSubtractor->apply(frame, fgMask);
            cv::erode(fgMask, fgMask, cv::Mat(), cv::Point(-1, -1), 2);
            cv::dilate(fgMask, fgMask, cv::Mat(), cv::Point(-1, -1), 2);

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(fgMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            for (const auto& contour : contours) {
                if (cv::contourArea(contour) < 100) continue;

                cv::Rect2d box = cv::boundingRect(contour);
                std::cout<<"new_box: "<<box.x<<" " << " " <<box.y<<std::endl;

                bool valid = true;
                for (const auto& obj : multiTracker->getObjects()) {
                    std::cout<<obj.x <<" "<<" " << obj.y<<std::endl;
                    valid = valid && !areRectsSimilar(obj, box);
                }
                std::cout<<"valid: " <<valid<<std::endl;

                if (valid) {

                    // Initialize CSRT tracker for each detected object
                    multiTracker->add(cv::legacy::TrackerCSRT::create(), frame, box);

                }
            }

            initialized = true;
            count += 1;
        }
        else {
            count += 1;
            // Update all trackers
            multiTracker->update(frame);
            std::cout<<multiTracker->getObjects().size()<<std::endl;
        }

        std::ostringstream ss;
        ss << std::fixed << std::setprecision(10) << "{\"lat\": \"" << "0" << "\", " << "\"lon\": \"" << "0" << "\"}";
        std::cout<<"sending: "<<ss.str()<<std::endl;
        df.send_message(ss.str().c_str(), ss.str().size());

        // Draw updated boxes
        for (const auto& obj : multiTracker->getObjects()) {
            cv::rectangle(frame, obj, cv::Scalar(0, 255, 0), 2);

            cv::Point2f center;
                center.x = obj.x + obj.width / 2.0;
                center.y = obj.y + obj.height / 2.0;
                std::vector<cv::Point2f> objectInImage = { center };
                std::vector<cv::Point2f> objectInWorld;

                cv::perspectiveTransform(objectInImage, objectInWorld, H);

                std::cout << "Real-world position: " 
                        << objectInWorld[0].x + relative_x << ", " 
                        << relative_y - objectInWorld[0].y << std::endl;

                double deltaLat = ((relative_y - objectInWorld[0].y) / earthRadius) * (180.0 / CV_PI);
                double deltaLon = ((objectInWorld[0].x + relative_x) / (earthRadius * cos(camera_lattitude * CV_PI / 180.0))) * (180.0 / CV_PI);

                rotate_point(deltaLon, deltaLat, -65);
                double current_lat = camera_lattitude + deltaLat;
                double current_long = camera_longitude + deltaLon;

                std::cout << "Real-world position: " 
                        << std::fixed << std::setprecision(15)
                        << current_lat << ", " 
                        << current_long << std::endl;

                std::ostringstream ss;
                ss << std::fixed << std::setprecision(10) << "{\"lat\": \"" << current_lat << "\", " << "\"lon\": \"" << current_long << "\"}";
                std::cout<<"sending: "<<ss.str()<<std::endl;
                df.send_message(ss.str().c_str(), ss.str().size());

        }

        cv::imshow("CSRT Tracking", frame);
        if (cv::waitKey(30) == 27) break;

        auto end = std::chrono::high_resolution_clock::now();
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

            if (elapsed_ms < frame_time_ms) {
                std::this_thread::sleep_for(std::chrono::milliseconds(frame_time_ms - elapsed_ms));
            }

    }

        return;
    // });
}