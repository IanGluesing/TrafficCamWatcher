#include "video_stream.h"
#include <thread>


void VideoStream::start() {
    auto x = std::thread([&](){
        if (!cap.isOpened()) {
            std::cerr << "Error: Could not open stream: " << cap.getBackendName() << std::endl;
            return;
        }

        double fps = 15.0;
        int frame_time_ms = static_cast<int>(1000.0 / fps);

        cv::Ptr<cv::BackgroundSubtractor> pBackSub = cv::createBackgroundSubtractorMOG2();
        cv::Mat prev_gray, motion_mask, mhi;
        double MHI_DURATION = 1; // in seconds
        double timestamp = 0;
        int frame_count = 0;

        cv::Mat fgMask;
        while (true) {
            auto start = std::chrono::high_resolution_clock::now();

            if (!cap.read(current_frame)) break;

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