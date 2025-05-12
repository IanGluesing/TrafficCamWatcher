#include "image_processing.h"
#include <opencv2/opencv.hpp>

image_processing::image_processing() :
    data_handler("ipc:///tmp/to_image_processing", zmq::socket_type::router) {
    
    // Create connection to server
    df = data_forwarder("ipc:///tmp/to_camera_server", zmq::socket_type::dealer);
    df.connect();

    // Parse camera json for list of active cameras
    std::vector<Video_Camera*> list_of_cameras_from_json;
    parse_camera_json(list_of_cameras_from_json);
    
    // Insert those cameras into camera map
    for (const auto& base_camera_impl: list_of_cameras_from_json) {
        camera_url_to_camera_info_map.insert({base_camera_impl->get_url(), base_camera_impl});
    }

    received_frame_count = 0;
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
    received_frame_count += 1;
    // cv::imshow(topic.c_str(), img);
    // if (cv::waitKey(30) == 27) return;
    std::cout<<received_frame_count<<std::endl;
}
