#include "nms_multi_tracker.h"


#include <opencv2/core/ocl.hpp>

#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

nms_multi_tracker::nms_multi_tracker(Video_Camera* camera_to_process) :
    base_image_tracker(camera_to_process),
    cv::legacy::MultiTracker() {

    calculated_homography = cv::findHomography(camera_to_process->get_camera_image_points(), camera_to_process->get_camera_world_points());
    cv::ocl::setUseOpenCL(true);
}

double calculate_IoU(const cv::Rect2d& box1, const cv::Rect2d& box2) {
    // Compute intersection rectangle
    cv::Rect2d intersection = box1 & box2;

    // If there's no intersection
    if (intersection.area() <= 0)
        return 0.0;

    // Compute union area
    double unionArea = box1.area() + box2.area() - intersection.area();

    // IoU is intersection over union
    return intersection.area() / unionArea;
}

void nms_multi_tracker::start() {
    zmq::message_t identity;
    zmq::message_t payload;

    receive_data.receive(identity);
    receive_data.receive(payload);
    
    std::vector<uchar> buf(payload.size());
    std::memcpy(buf.data(), payload.data(), payload.size());
    previous_frame = cv::imdecode(buf, cv::IMREAD_UNCHANGED);
    
    cv::cvtColor(previous_frame, previous_frame, cv::COLOR_RGB2GRAY);

    data_handler::start();
}

inline void nms_multi_tracker::process_msg(
    const zmq::message_t& identity, 
    const zmq::message_t& payload) {

    // Read Frame from zeromq message
    std::vector<uchar> buf(payload.size());
    std::memcpy(buf.data(), payload.data(), payload.size());
    cv::Mat current_frame = cv::imdecode(buf, cv::IMREAD_UNCHANGED);
    cv::cvtColor(current_frame, current_frame, cv::COLOR_RGB2GRAY);

    diff = current_frame - previous_frame;
    cv::medianBlur(diff, diff, 3);
    cv::adaptiveThreshold(diff, diff, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 11, 3);
    cv::medianBlur(diff, diff, 3);
    cv::morphologyEx(diff, diff, cv::MORPH_CLOSE, 3);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(diff, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_TC89_L1);

    std::vector<cv::Rect2d> rectangles;
    for (const auto& points : contours) {
        // Add bounding box if area is large enough based on the contour detected
        if (cv::boundingRect(points).area() < 400) continue;
        rectangles.push_back(cv::boundingRect(points));
    }

    // Sort current bounding boxes for this frame in descending order
    std::sort(rectangles.begin(), rectangles.end(), [](const cv::Rect2d& a, const cv::Rect2d& b) {
        return a.area() > b.area();
    });
    
    // Remove any boxes that are entirely contained in another box
    for (int i = 0; i < rectangles.size(); i++) {
        auto current_box = rectangles[i];
        for (int j = i + 1; j < rectangles.size();) {
            auto smaller_box = rectangles[j];
            // Remove if area is equal to the area of the smaller box
            // This can be modified to remove any smaller boxes that intersect a larger box
            if ((current_box & smaller_box).area() == smaller_box.area()) {
                std::cout<<"erasing"<<std::endl;
                rectangles.erase(rectangles.begin() + j);
            } else {
                j += 1;
            }
        }
    }
    
    // Remove any boxes that intersect an already existing box
    for (int i = 0; i < current_trackers.size(); i++) {
        auto current_box = current_trackers[i].bounding_box;
        for (int j = 0; j < rectangles.size();) {
            auto smaller_box = rectangles[j];
            // Remove if area is equal to the area of the smaller box
            // This can be modified to remove any smaller boxes that intersect a larger box
            if ((current_box & smaller_box).area() != 0) {
                // std::cout<<"erasing"<<std::endl;
                rectangles.erase(rectangles.begin() + j);
            } else {
                j += 1;
            }
        }
    }

    std::vector<cv::Rect2d> keep;
    for (int i = 0; i < rectangles.size(); i++) {
        auto current_box = rectangles[i];
        keep.push_back(current_box);
        for (int j = i + 1; j < rectangles.size();) {
            auto smaller_box = rectangles[j];

            auto calc = calculate_IoU(current_box, smaller_box);
            if (calc > .01) {
                rectangles.erase(rectangles.begin() + j);
            } else {
                j += 1;
            }
        }
    }

    // Sort current bounding boxes for this frame in descending order
    std::sort(current_trackers.begin(), current_trackers.end(), [](const custom_tracker& a, const custom_tracker& b) {
        return a.bounding_box.area() > b.bounding_box.area();
    });
    for (int i = 0; i < current_trackers.size(); i++) {
        auto current_box = current_trackers[i].bounding_box;
        for (int j = i + 1; j < current_trackers.size();) {
            auto smaller_box = current_trackers[j].bounding_box;
            // Remove any trackers that currently overlap another
            if ((current_box & smaller_box).area() != 0) {
                // std::cout<<"erasing"<<std::endl;
                current_trackers.erase(current_trackers.begin() + j);
            } else {
                j += 1;
            }
        }
    }

    std::cout<<"current trackers: " <<current_trackers.size()<<std::endl;
    for (int i = 0; i < current_trackers.size();) {
        if (!current_tracker_still_valid(i)) {
            std::cout<<"erasing tracker"<<std::endl;
            current_trackers.erase(current_trackers.begin() + i);
        } else {
            i += 1;
        }
    }

    cv::Mat rect_frame = current_frame.clone();
    for (const auto& obj : keep) {
        auto to_insert = custom_tracker {
            cv::legacy::TrackerCSRT::create(),
            obj,
            std::sqrt((obj.x * obj.x) + (obj.y * obj.y)),
            0,
            0
        };

        to_insert.tracker_impl->init(diff, obj);

        current_trackers.push_back(to_insert);
        // current_trackers
        // add(cv::legacy::TrackerCSRT::create(), diff, obj);
        // centroid_displacement_vs_last_frame.push_back(std::sqrt((obj.x * obj.x) + (obj.y * obj.y)));
        // insufficient_movement_frames_in_a_row.push_back(0);
        // failed_updates_in_a_row.push_back(0);
    }

    for (const auto& custom_tracker_struct : current_trackers) {
        cv::rectangle(rect_frame, custom_tracker_struct.bounding_box, cv::Scalar(0, 255, 255), 2);
    }

    // // Send reset frame for current camera
    // send_lat_long_pair(0, 0);

    // // Draw updated boxes
    // for (const auto& obj : multi_tracker->getObjects()) {
    //     // Draw rectangle of current object on current frame
    //     cv::rectangle(frame, obj, cv::Scalar(0, 255, 0), 2);

    //     // Create center point of the object in the image
    //     cv::Point2f center;
    //     center.x = obj.x + obj.width / 2.0;
    //     center.y = obj.y + obj.height / 2.0;
    //     std::vector<cv::Point2f> objectInImage = { center };

    //     // Use perspective transform to convert from image point to estimated real world position
    //     // relative to the point of interest 0, 0 image point
    //     std::vector<cv::Point2f> objectInWorld;
    //     cv::perspectiveTransform(objectInImage, objectInWorld, calculated_homography);

    //     // Convert from point of interest 0, 0 coordinate frame to the camera coordinate frame
    //     // objectInWorld will now store the offset between the camera and itself, stored in meters
    //     objectInWorld[0].y = camera_to_process->get_camera_relative_y() - objectInWorld[0].y;
    //     objectInWorld[0].x = objectInWorld[0].x - camera_to_process->get_camera_relative_x();

    //     // Convert the meter difference to lattitude longitude offsets
    //     double deltaLat = (objectInWorld[0].y / earth_radius) * (180.0 / CV_PI);
    //     double deltaLon = (objectInWorld[0].x / (earth_radius * cos(camera_to_process->get_camera_lattitude() * CV_PI / 180.0))) * (180.0 / CV_PI);

    //     // Rotate offset vector in the approximate direction the camera is facing
    //     // relative to true north 
    //     rotate_point(deltaLon, deltaLat, camera_to_process->get_camera_rotation());

    //     // Real world absolute coordinate is the camera position plus the rotated offsets
    //     double point_lattitude = camera_to_process->get_camera_lattitude() + deltaLat;
    //     double point_longitude = camera_to_process->get_camera_longitude() + deltaLon;

    //     send_lat_long_pair(point_lattitude, point_longitude);
    // }

    previous_frame = current_frame.clone();
    display_frame = rect_frame.clone();
}

bool nms_multi_tracker::current_tracker_still_valid(int i) {
    auto& custom_tracker_struct = current_trackers[i];

    auto previous_x = custom_tracker_struct.bounding_box.x;
    auto previous_y = custom_tracker_struct.bounding_box.y;
    auto previous_centroid_displacement = custom_tracker_struct.centroid_displacement_vs_last_frame;

    if (custom_tracker_struct.tracker_impl->update(diff, custom_tracker_struct.bounding_box)) {
        custom_tracker_struct.failed_updates_in_a_row = 0;
    // } else if (custom_tracker_struct.tracker_impl->update(current_frame, custom_tracker_struct.bounding_box)) {
    //     custom_tracker_struct.failed_updates_in_a_row = 0;
    } else {
        custom_tracker_struct.failed_updates_in_a_row += 1;
    }

    custom_tracker_struct.centroid_displacement_vs_last_frame = (std::sqrt((custom_tracker_struct.bounding_box.x - previous_x) + (custom_tracker_struct.bounding_box.y - previous_y)));

    if (std::abs(previous_centroid_displacement - custom_tracker_struct.centroid_displacement_vs_last_frame) < 1) {
        custom_tracker_struct.insufficient_movement_frames_in_a_row += 1;
    } else {
        custom_tracker_struct.insufficient_movement_frames_in_a_row = 0;
    }

    if (custom_tracker_struct.failed_updates_in_a_row >= 5 || custom_tracker_struct.insufficient_movement_frames_in_a_row >= 5) {
        return false;
    }

    return true;
}
