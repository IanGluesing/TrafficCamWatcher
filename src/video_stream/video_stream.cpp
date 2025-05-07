#include "video_stream.h"
#include <thread>

using namespace cv;

void VideoStream::start() {
    auto x = std::thread([&](){
        if (!cap.isOpened()) {
            std::cerr << "Error: Could not open stream: " << cap.getBackendName() << std::endl;
            return;
        }

        double fps = cap.get(cv::CAP_PROP_FPS);
        int frame_time_ms = static_cast<int>(1000.0 / fps);

        Ptr<BackgroundSubtractor> pBackSub = createBackgroundSubtractorKNN();

        cv::Mat fgMask;
        cv::Mat input_frame;
        while (true) {
            auto start = std::chrono::high_resolution_clock::now();

            if (!cap.read(input_frame)) break;

            pBackSub->apply(input_frame, fgMask);

            current_frame = fgMask.clone();

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