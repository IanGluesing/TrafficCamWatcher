#include "non_maximum_suppression.h"

#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

non_maximum_suppression::non_maximum_suppression(Video_Camera* camera_to_process) :
    base_image_tracker(camera_to_process) {
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

void non_maximum_suppression::start() {
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

inline void non_maximum_suppression::process_msg(
    const zmq::message_t& identity, 
    const zmq::message_t& payload) {

    // Read Frame from zeromq message
    std::vector<uchar> buf(payload.size());
    std::memcpy(buf.data(), payload.data(), payload.size());
    cv::Mat current_frame = cv::imdecode(buf, cv::IMREAD_UNCHANGED);
    cv::Mat current_frame_gray;
    cv::cvtColor(current_frame, current_frame_gray, cv::COLOR_RGB2GRAY);

    diff = current_frame_gray - previous_frame;
    cv::medianBlur(diff, diff, 3);
    cv::adaptiveThreshold(diff, diff, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 11, 3);
    cv::medianBlur(diff, diff, 3);
    cv::morphologyEx(diff, diff, cv::MORPH_CLOSE, 2);
    cv::morphologyEx(diff, diff, cv::MORPH_DILATE, 1);

    std::vector<std::vector<cv::Point>> contours;
    // Find original contours
    cv::findContours(diff, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_TC89_L1);

    std::vector<cv::Rect2d> rectangles;
    for (const auto& points : contours) {
        // Add bounding box to list if the bounding box of the contour is large enough
        if (cv::boundingRect(points).area() < 300) continue;
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
                rectangles.erase(rectangles.begin() + j);
            } else {
                j += 1;
            }
        }
    }

    // Keep all rectangles that meet the IoU threshold
    std::vector<cv::Rect2d> final_rectangles;
    for (int i = 0; i < rectangles.size(); i++) {
        auto current_box = rectangles[i];
        final_rectangles.push_back(current_box);
        for (int j = i + 1; j < rectangles.size();) {
            auto smaller_box = rectangles[j];

            auto iou_of_boxes = calculate_IoU(current_box, smaller_box);
            if (iou_of_boxes > .1) {
                rectangles.erase(rectangles.begin() + j);
            } else {
                j += 1;
            }
        }
    }

    cv::Mat rect_frame = current_frame.clone();
    for (const auto& obj : final_rectangles) {
        cv::rectangle(rect_frame, obj, cv::Scalar(0, 255, 255), 2);
    }

    // Send reset frame for current camera
    send_lat_long_pair(0, 0);

    send_point_list(final_rectangles);

    previous_frame = current_frame_gray.clone();
    display_frame = rect_frame.clone();
}