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

Video_Camera::Video_Camera(
    const std::string& url,
    double camera_lattitude,
    double camera_longitude,
    const std::vector<cv::Point2f>& imagePoints,
    const std::vector<cv::Point2f>& worldPoints,
    const double relative_y,
    const double relative_x,
    const double rotation) : 
    base_camera(
        url,
        camera_lattitude,
        camera_longitude,
        imagePoints,
        worldPoints,
        relative_y,
        relative_x,
        rotation)
{
}

Video_Camera::~Video_Camera() {
    if (camera_thread.joinable()) {
        camera_thread.join();
    }
}

void Video_Camera::start_thread() {
    std::cout<<"eh"<<std::endl;
    this->camera_thread = std::thread([&](){ 
        std::cout<<"eh"<<std::endl;
        data_forwarder df = data_forwarder("ipc:///tmp/to_image_processing", zmq::socket_type::dealer);
        df.get_socket().set(zmq::sockopt::routing_id, url);
        df.connect();

        // cv::Mat H = cv::findHomography(imagePoints, worldPoints);

        // cv::ocl::setUseOpenCL(true);
        std::cout<<url<<std::endl;
        cv::VideoCapture cap(url); 
        if (!cap.isOpened()) {
            std::cerr << "Error opening video stream" << std::endl;
            return;
        }

        double fps = cap.get(cv::CAP_PROP_FPS);
        int frame_time_ms = static_cast<int>(1000.0 / fps);


        // cv::Ptr<cv::BackgroundSubtractor> bgSubtractor = cv::createBackgroundSubtractorMOG2();
        cv::Mat frame, fgMask;
        // cap >> frame;

    // cv::Ptr<cv::legacy::MultiTracker> multiTracker = cv::legacy::MultiTracker::create();

    bool initialized = false;
    int count = 0;
    while (true) {
        auto start = std::chrono::high_resolution_clock::now();

        cap >> frame;
        if (frame.empty()) break;

        std::ostringstream ss2;
        size_t size = frame.total() * frame.elemSize();
        std::vector<uchar> buffer;
        cv::imencode(".png", frame, buffer);
        ss2 << "{"
            << "\"rows\": \"" << frame.rows << "\", " 
            << "\"cols\": \"" << frame.cols << "\", "
            << "\"data\": \"" << buffer.data() << "\""
            << "}";

        // std::string topic_message("CAMERA_1");
        // zmq::message_t topic(topic_message.data(), topic_message.size());
        // df.send_message(topic, zmq::send_flags::sndmore);

        std::cout<<"sending: "<<ss2.str()<<std::endl;
        df.send_message(buffer.data(), buffer.size());
        std::cout<<++count<<std::endl;

        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        if (elapsed_ms < frame_time_ms) {
            std::this_thread::sleep_for(std::chrono::milliseconds(frame_time_ms - elapsed_ms));
        }

        // continue;

        // if (!initialized || ((count+1) % 5 == 0)) {
        //     if (((count+1) % 10) == 0) {
        //         multiTracker = cv::legacy::MultiTracker::create();
        //     }
        //     bgSubtractor->apply(frame, fgMask);
        //     cv::erode(fgMask, fgMask, cv::Mat(), cv::Point(-1, -1), 2);
        //     cv::dilate(fgMask, fgMask, cv::Mat(), cv::Point(-1, -1), 2);

        //     std::vector<std::vector<cv::Point>> contours;
        //     cv::findContours(fgMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            // for (const auto& contour : contours) {
            //     if (cv::contourArea(contour) < 100) continue;

            //     cv::Rect2d box = cv::boundingRect(contour);
            //     std::cout<<"new_box: "<<box.x<<" " << " " <<box.y<<std::endl;

            //     bool valid = true;
            //     for (const auto& obj : multiTracker->getObjects()) {
            //         std::cout<<obj.x <<" "<<" " << obj.y<<std::endl;
            //         valid = valid && !areRectsSimilar(obj, box);
            //     }
            //     std::cout<<"valid: " <<valid<<std::endl;

            //     if (valid) {

            //         // Initialize CSRT tracker for each detected object
            //         multiTracker->add(cv::legacy::TrackerCSRT::create(), frame, box);

            //     }
            // }

            // initialized = true;
        //     count += 1;
        // }
        // else {
        //     count += 1;
        //     // Update all trackers
        //     multiTracker->update(frame);
        //     std::cout<<multiTracker->getObjects().size()<<std::endl;
        // }

        // std::ostringstream ss;
        // ss << std::fixed << std::setprecision(10) << "{\"lat\": \"" << "0" << "\", " << "\"lon\": \"" << "0" << "\"}";
        // std::cout<<"sending: "<<ss.str()<<std::endl;
        // df.send_message(ss.str().c_str(), ss.str().size());

        // // Draw updated boxes
        // for (const auto& obj : multiTracker->getObjects()) {
        //     cv::rectangle(frame, obj, cv::Scalar(0, 255, 0), 2);

        //     cv::Point2f center;
        //         center.x = obj.x + obj.width / 2.0;
        //         center.y = obj.y + obj.height / 2.0;
        //         std::vector<cv::Point2f> objectInImage = { center };
        //         std::vector<cv::Point2f> objectInWorld;

        //         cv::perspectiveTransform(objectInImage, objectInWorld, H);

        //         std::cout << "Real-world position: " 
        //                 << objectInWorld[0].x + relative_x << ", " 
        //                 << relative_y - objectInWorld[0].y << std::endl;

        //         double deltaLat = ((relative_y - objectInWorld[0].y) / earthRadius) * (180.0 / CV_PI);
        //         double deltaLon = ((objectInWorld[0].x + relative_x) / (earthRadius * cos(camera_lattitude * CV_PI / 180.0))) * (180.0 / CV_PI);

        //         rotate_point(deltaLon, deltaLat, rotation);
        //         double current_lat = camera_lattitude + deltaLat;
        //         double current_long = camera_longitude + deltaLon;

        //         std::cout << "Real-world position: " 
        //                 << std::fixed << std::setprecision(15)
        //                 << current_lat << ", " 
        //                 << current_long << std::endl;

        //         std::ostringstream ss;
        //         ss << std::fixed << std::setprecision(10) << "{\"lat\": \"" << current_lat << "\", " << "\"lon\": \"" << current_long << "\"}";
        //         std::cout<<"sending: "<<ss.str()<<std::endl;
        //         df.send_message(ss.str().c_str(), ss.str().size());

        // }

        // cv::imshow("CSRT Tracking", frame);
        // if (cv::waitKey(30) == 27) break;

        // auto end = std::chrono::high_resolution_clock::now();
        // auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        // if (elapsed_ms < frame_time_ms) {
        //     std::this_thread::sleep_for(std::chrono::milliseconds(frame_time_ms - elapsed_ms));
        // }

    }

        return;
    });

    this->camera_thread.detach();
}

void parse_camera_json(std::vector<Video_Camera*>& list_of_cameras_from_json) {
    // Read camera information
    std::ifstream file(CAMERA_JSON_FILE_PATH);
    if (!file.is_open()) {
        std::cerr << "Failed to open JSON file." << std::endl;
        return;
    }

    // Read entire file into a string
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string json_content = buffer.str();

    // Parse JSON string into fjson object
    flatjson::fjson json(json_content.data(), json_content.size());
    
    for (int i = 0; i < json.size(); i++){
        auto elem = json.at(i);

        std::string camera_url = elem["camera_url"].to_string();
        double camera_lattitude = elem["camera_lattitude"].to_double();
        double camera_longitude = elem["camera_longitude"].to_double();
        // parse image points
        std::vector<cv::Point2f> image_points;

        // parse world points
        std::vector<cv::Point2f> world_points;

        double camera_y_position_wrt_0_0 = elem["camera_y_position_wrt_0_0"].to_double();
        double camera_x_position_wrt_0_0 = elem["camera_x_position_wrt_0_0"].to_double();
        double camera_rotation = elem["camera_rotation"].to_double();
        
        Video_Camera* element = new Video_Camera(
            camera_url,
            camera_lattitude,
            camera_longitude,
            image_points,
            world_points,
            camera_y_position_wrt_0_0,
            camera_x_position_wrt_0_0,
            camera_rotation
        );

        list_of_cameras_from_json.push_back(element);
    }
}