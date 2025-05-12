#include "image_processing.h"

#include <opencv2/tracking.hpp>

#include <opencv2/core/ocl.hpp>
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

image_processing::image_processing(Video_Camera* camera_to_process) :
    data_handler(std::string("ipc:///tmp/" + std::to_string(camera_to_process->get_camera_lattitude())).c_str(), zmq::socket_type::sub) {
    
    receive_data.get_socket().set(zmq::sockopt::subscribe, camera_to_process->get_url());

    // Create connection to server
    df = data_forwarder("ipc:///tmp/to_camera_server", zmq::socket_type::dealer);
    df.connect();

    this->camera_to_process = camera_to_process;

    received_frame_count = 0;
}

image_processing::~image_processing() {

}

inline void image_processing::process_msg(
    const zmq::message_t& identity, 
    const zmq::message_t& payload) {

    std::string topic(identity.to_string());
    
    std::vector<uchar> buf(payload.size());
    std::memcpy(buf.data(), payload.data(), payload.size());

    cv::Mat img = cv::imdecode(buf, cv::IMREAD_UNCHANGED);
    received_frame_count += 1;
    // cv::imshow(topic.c_str(), img);
    // if (cv::waitKey(30) == 27) return;
    std::cout<<received_frame_count<<std::endl;
}

void image_processing::start() {
    // ADD THREAD HERE
    cv::Mat H = cv::findHomography(camera_to_process->get_camera_image_points(), camera_to_process->get_camera_world_points());
    cv::ocl::setUseOpenCL(true);
    cv::Ptr<cv::BackgroundSubtractor> bgSubtractor = cv::createBackgroundSubtractorMOG2();
    cv::Ptr<cv::legacy::MultiTracker> multiTracker = cv::legacy::MultiTracker::create();
    bool initialized = false;
    int count = 0;
    cv::Mat fgMask;

    while (true) {
        zmq::message_t identity;
        zmq::message_t payload;

        receive_data.receive(identity);
        receive_data.receive(payload);

        // Get start time for processing current frame
        auto start = std::chrono::high_resolution_clock::now();

        std::string topic(identity.to_string());

        // Read Frame from zeromq message
        std::vector<uchar> buf(payload.size());
        std::memcpy(buf.data(), payload.data(), payload.size());
        cv::Mat frame = cv::imdecode(buf, cv::IMREAD_UNCHANGED);

        received_frame_count += 1;
        std::cout<<received_frame_count<<std::endl;

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

                bool valid = true;
                for (const auto& obj : multiTracker->getObjects()) {
                    valid = valid && !areRectsSimilar(obj, box);
                }

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
        }

        // Send reset frame for now
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(10) << "{\"lat\": \"" << "0" << "\", " << "\"lon\": \"" << "0" << "\"}";
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

            double deltaLat = ((camera_to_process->get_camera_relative_y() - objectInWorld[0].y) / earthRadius) * (180.0 / CV_PI);
            double deltaLon = ((objectInWorld[0].x - camera_to_process->get_camera_relative_x()) / (earthRadius * cos(camera_to_process->get_camera_lattitude() * CV_PI / 180.0))) * (180.0 / CV_PI);

            rotate_point(deltaLon, deltaLat, camera_to_process->get_camera_rotation());
            double current_lat = camera_to_process->get_camera_lattitude() + deltaLat;
            double current_long = camera_to_process->get_camera_longitude() + deltaLon;

            std::ostringstream ss;
            ss << std::fixed << std::setprecision(10) << "{\"lat\": \"" << current_lat << "\", " << "\"lon\": \"" << current_long << "\"}";
            df.send_message(ss.str().c_str(), ss.str().size());
        }

        // cv::imshow("test", frame);
        // if (cv::waitKey(30) == 27) break;

        // End statistics
        auto end = std::chrono::high_resolution_clock::now();
        auto processing_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        // std::cout << "Processing time: " << processing_time << " µs" << std::endl;
    }
}
