
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>
#include <opencv2/optflow/motempl.hpp>

using namespace cv;

std::unordered_map<std::string, cv::Mat> my_map = {
    {"https://iowadotsfs1.us-east-1.skyvdn.com:443/rtplive/ictv03lb/playlist.m3u8", cv::Mat()},
    {"https://iowadotsfs2.us-east-1.skyvdn.com:443/rtplive/ictv38lb/playlist.m3u8", cv::Mat()},
    {"https://iowadotsfs1.us-east-1.skyvdn.com:443/rtplive/dmtv44qlb/playlist.m3u8", cv::Mat()}
};

std::mutex m;

void mhi(cv::VideoCapture& cap, const std::string& url) {
    double fps = 15.0;
	int frame_time_ms = static_cast<int>(1000.0 / fps);

    cv::Ptr<cv::BackgroundSubtractor> pBackSub = cv::createBackgroundSubtractorMOG2();
	cv::Mat prev_gray, motion_mask, mhi;
	double MHI_DURATION = 1; // in seconds
	double timestamp = 0;
	int frame_count = 0;

    cv::Mat frame, fgMask;
    while (true) {
        auto start = std::chrono::high_resolution_clock::now();

	    if (!cap.read(frame)) break;

        {
            std::lock_guard<std::mutex> lg(m);
            my_map[url] = frame.clone();
        }

	    auto end = std::chrono::high_resolution_clock::now();
	    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

	    if (elapsed_ms < frame_time_ms) {
	        std::this_thread::sleep_for(std::chrono::milliseconds(frame_time_ms - elapsed_ms));
	    }

    }

}

void launch(const std::string url) {
    auto x = std::thread([url]() {
        // Open the video stream
        cv::VideoCapture cap(url);
        if (!cap.isOpened()) {
            std::cerr << "Error: Could not open stream: " << cap.getBackendName() << std::endl;
            return -1;
        }

        mhi(cap, url);

        cap.release();
        cv::destroyAllWindows();

        return 0;
    });

    x.detach();

    return;
}

int main(int argc, char* argv[]) {

    for (const auto& pair: my_map) {
        launch(pair.first);
    }


    while (true) {
        for(const auto& my_pair: my_map) {
            if (my_pair.second.empty()) continue;
            imshow(my_pair.first, my_pair.second);
        }
        cv::waitKey(1);
    }
    

    return 0;
}
