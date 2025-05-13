
#include <opencv2/opencv.hpp>
#include <iostream>

void onMouse(int event, int x, int y, int, void*) {
    if (event == cv::EVENT_MOUSEMOVE) {
        std::cout << "Mouse Position: (" << x << ", " << y << ")" << std::endl;
    }
}

int main() {

    cv::VideoCapture cap("https://iowadotsfs2.us-east-1.skyvdn.com:443/rtplive/ictv39lb/playlist.m3u8"); 
    if (!cap.isOpened()) {
        std::cerr << "Error opening video stream" << std::endl;
        return 0;
    }

    cv::Mat frame;
    cap >> frame;
    
    cv::namedWindow("Image Window", cv::WINDOW_AUTOSIZE);

    // Set mouse callback
    cv::setMouseCallback("Image Window", onMouse, nullptr);

    // Display loop
    while (true) {
        cv::imshow("Image Window", frame);

        // Break on ESC key
        if (cv::waitKey(1) == 27) break;
    }

    cv::destroyAllWindows();
    return 0;
}
