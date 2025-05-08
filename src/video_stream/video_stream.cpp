#include "video_stream.h"
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <thread>

using namespace cv;

void VideoStream::start() {
    auto x = std::thread([&](){
        cv::ocl::setUseOpenCL(true);

        if (!cap.isOpened()) {
            std::cerr << "Error: Could not open stream: " << cap.getBackendName() << std::endl;
            return;
        }

        double fps = cap.get(cv::CAP_PROP_FPS);
        int frame_time_ms = static_cast<int>(1000.0 / fps);

        cv::Mat input_frame;
        cap.read(input_frame);
        cap.read(input_frame);

        cv::Rect2d bbox = cv::selectROI("Select Object", input_frame, false, false);
        cv::Rect bbox_int = bbox;
        cv::destroyWindow("Select Object");
        cv::Ptr<cv::Tracker> tracker = cv::TrackerCSRT::create();
        if (tracker.empty()) {
            std::cerr << "Failed to create tracker!" << std::endl;
            return;
        }
        tracker->init(input_frame, bbox);

        while (true) {
            auto start = std::chrono::high_resolution_clock::now();

            if (!cap.read(input_frame)) break;

            bool success = tracker->update(input_frame, bbox_int);

            if (success) {
                // Tracking success — draw rectangle
                cv::rectangle(input_frame, bbox, cv::Scalar(0, 255, 0), 2, 1);
                cv::putText(input_frame, "Tracking", cv::Point(20, 40),
                            cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
            } else {
                // Tracking failure
                cv::putText(input_frame, "Lost Tracking", cv::Point(20, 40),
                            cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
            }

            current_frame = input_frame.clone();

            auto end = std::chrono::high_resolution_clock::now();
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

            if (elapsed_ms < frame_time_ms) {
                std::this_thread::sleep_for(std::chrono::milliseconds(frame_time_ms - elapsed_ms));
            }

        }

        cap.release();
        cv::destroyAllWindows();

    });

    x.detach();

    return;
}