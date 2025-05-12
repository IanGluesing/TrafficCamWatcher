#include "image_processing.h"
#include <opencv2/opencv.hpp>
#include "flatjson.hpp"


image_processing::image_processing() :
    data_handler("ipc:///tmp/to_image_processing", zmq::socket_type::router) {
    
    // Create connection to server
    df = data_forwarder("ipc:///tmp/to_camera_server", zmq::socket_type::dealer);
    df.connect();
}

image_processing::~image_processing() {

}

inline void image_processing::process_msg(
    const zmq::message_t& identity, 
    const zmq::message_t& payload) {

    std::string topic(identity.to_string());
    
    std::vector<uchar> buf(payload.size());
    std::memcpy(buf.data(), payload.data(), payload.size());

    cv::Mat img = cv::imdecode(buf, cv::IMREAD_UNCHANGED);

    cv::imshow(topic.c_str(), img);
    if (cv::waitKey(30) == 27) return;
}
