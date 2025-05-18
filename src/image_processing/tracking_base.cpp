#include "tracking_base.h"

#include <opencv2/core/ocl.hpp>

#include "tracking_utils.h"

base_image_tracker::base_image_tracker(Video_Camera* camera_to_process) :
    data_handler(std::string("ipc:///tmp/" + std::to_string(camera_to_process->get_camera_lattitude())).c_str(), zmq::socket_type::sub) {
    
    // Initialize receiver socket
    receive_data.get_socket().set(zmq::sockopt::subscribe, camera_to_process->get_url());

    // Create connection to server
    df = data_forwarder("ipc:///tmp/to_camera_server", zmq::socket_type::dealer);
    df.connect();

    this->camera_to_process = camera_to_process;

    received_frame_count = 0;

    calculated_homography = cv::findHomography(camera_to_process->get_camera_image_points(), camera_to_process->get_camera_world_points());
    cv::ocl::setUseOpenCL(true);

}

void base_image_tracker::send_lat_long_pair(const double& point_lattitude, const double& point_longitude) {
    // Create ostringstream
    std::ostringstream ss;

    // Pack message
    ss << std::fixed << std::setprecision(10) 
        << "{\"lat\": \"" << point_lattitude << "\", " 
        << "\"lon\": \"" << point_longitude << "\", " 
        << "\"camera_name\": \"" << camera_to_process->get_url()
        << "\"}";

    // Send message to receiver
    df.send_message(ss.str().c_str(), ss.str().size());
}

void base_image_tracker::send_point_list(const std::vector<cv::Rect2d> list_of_points) {
    for (const auto& obj : list_of_points) {
        // Create center point of the object in the image
        cv::Point2f center;
        center.x = obj.x + obj.width / 2.0;
        center.y = obj.y + obj.height / 2.0;
        std::vector<cv::Point2f> objectInImage = { center };

        // Use perspective transform to convert from image point to estimated real world position
        // relative to the point of interest 0, 0 image point
        std::vector<cv::Point2f> objectInWorld;
        cv::perspectiveTransform(objectInImage, objectInWorld, calculated_homography);

        // Convert from point of interest 0, 0 coordinate frame to the camera coordinate frame
        // objectInWorld will now store the offset between the camera and itself, stored in meters
        objectInWorld[0].y = camera_to_process->get_camera_relative_y() - objectInWorld[0].y;
        objectInWorld[0].x = objectInWorld[0].x - camera_to_process->get_camera_relative_x();

        // Convert the meter difference to lattitude longitude offsets
        double deltaLat = (objectInWorld[0].y / earth_radius) * (180.0 / CV_PI);
        double deltaLon = (objectInWorld[0].x / (earth_radius * cos(camera_to_process->get_camera_lattitude() * CV_PI / 180.0))) * (180.0 / CV_PI);

        // Rotate offset vector in the approximate direction the camera is facing
        // relative to true north 
        rotate_point(deltaLon, deltaLat, camera_to_process->get_camera_rotation());

        // Real world absolute coordinate is the camera position plus the rotated offsets
        double point_lattitude = camera_to_process->get_camera_lattitude() + deltaLat;
        double point_longitude = camera_to_process->get_camera_longitude() + deltaLon;

        send_lat_long_pair(point_lattitude, point_longitude);
    }

}