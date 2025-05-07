
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>
#include <opencv2/optflow/motempl.hpp>

#include <set>

#include "video_stream.h"

using namespace cv;

std::set<VideoStream*> stream_set = {
    new VideoStream("https://wzmedia.dot.ca.gov/D3/50_24th_St_SAC50_WB.stream/chunklist_w1659799971.m3u8") // https://wzmedia.dot.ca.gov/D3/50_24th_St_SAC50_WB.stream/playlist.m3u8
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
