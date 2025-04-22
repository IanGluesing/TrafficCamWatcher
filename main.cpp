
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>
#include <opencv2/optflow/motempl.hpp>

#include <set>

#include "video_stream.h"

using namespace cv;

std::set<VideoStream*> stream_set = {
    new VideoStream("https://iowadotsfs1.us-east-1.skyvdn.com:443/rtplive/ictv03lb/playlist.m3u8"),
    new VideoStream("https://iowadotsfs2.us-east-1.skyvdn.com:443/rtplive/ictv38lb/playlist.m3u8"),
    new VideoStream("https://iowadotsfs1.us-east-1.skyvdn.com:443/rtplive/dmtv44qlb/playlist.m3u8")
};

int main(int argc, char* argv[]) {

    for (auto& stream: stream_set) {
        stream->start();
    }

    while (true) {
        for(auto& stream: stream_set) {
            cv::Mat current_frame = stream->get_current_frame();
            if (current_frame.empty()) continue;
            imshow(stream->get_url(), current_frame);
        }
        cv::waitKey(1);
    }
    

    return 0;
}
