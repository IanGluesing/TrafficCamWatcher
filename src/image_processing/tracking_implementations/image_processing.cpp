#include "image_processing.h"

#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include "tracking_utils.h"

image_processing::image_processing(Video_Camera* camera_to_process) :
    base_image_tracker(camera_to_process) {

    background_subtractor = cv::createBackgroundSubtractorMOG2();

    multi_tracker = cv::legacy::MultiTracker::create();

    initialized = false;

    count = 0;
}

inline void image_processing::process_msg(
    const zmq::message_t& identity, 
    const zmq::message_t& payload) {

    // Read Frame from zeromq message
    std::vector<uchar> buf(payload.size());
    std::memcpy(buf.data(), payload.data(), payload.size());
    cv::Mat frame = cv::imdecode(buf, cv::IMREAD_UNCHANGED);

    // Increment frame count
    received_frame_count += 1;
    std::cout<<received_frame_count<<std::endl;

    if (frame.empty()) return;

    // On first frame or every 5 frames, reinitialize the tracker
    if (!initialized || ((count+1) % 5 == 0)) {
        if (((count+1) % 10) == 0) {
            multi_tracker = cv::legacy::MultiTracker::create();
        }

        // Update mask
        background_subtractor->apply(frame, fgMask);
        cv::erode(fgMask, fgMask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(fgMask, fgMask, cv::Mat(), cv::Point(-1, -1), 2);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(fgMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Filter contours
        for (const auto& contour : contours) {
            if (cv::contourArea(contour) < 100) continue;

            cv::Rect2d box = cv::boundingRect(contour);

            // New rectangle validity check
            bool valid = true;
            for (const auto& obj : multi_tracker->getObjects()) {
                valid = valid && !are_rectangles_similar(obj, box);
            }

            if (valid) {
                // Initialize CSRT tracker for each detected object
                multi_tracker->add(cv::legacy::TrackerCSRT::create(), frame, box);
            }
        }

        initialized = true;
        count += 1;
    }
    else {
        count += 1;
        // Update all trackers
        multi_tracker->update(frame);
    }

    // Send reset frame for current camera
    send_lat_long_pair(0, 0);

    // Draw updated boxes
    for (const auto& obj : multi_tracker->getObjects()) {
        // Draw rectangle of current object on current frame
        cv::rectangle(frame, obj, cv::Scalar(0, 255, 0), 2);

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

    display_frame = frame.clone();
}
