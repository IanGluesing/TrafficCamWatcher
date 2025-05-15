#include "tracking_base.h"

base_image_tracker::base_image_tracker(Video_Camera* camera_to_process) :
    data_handler(std::string("ipc:///tmp/" + std::to_string(camera_to_process->get_camera_lattitude())).c_str(), zmq::socket_type::sub) {
    
    // Initialize receiver socket
    receive_data.get_socket().set(zmq::sockopt::subscribe, camera_to_process->get_url());

    // Create connection to server
    df = data_forwarder("ipc:///tmp/to_camera_server", zmq::socket_type::dealer);
    df.connect();

    this->camera_to_process = camera_to_process;

    received_frame_count = 0;
}

void base_image_tracker::send_lat_long_pair(const double& point_lattitude, const double& point_longitude) {
    // Create ostringstream
    std::ostringstream ss;

    // Pack message
    ss << std::fixed << std::setprecision(10) 
        << "{\"lat\": \"" << point_lattitude << "\", " 
        << "\"lon\": \"" << point_longitude << "\", " 
        << "\"camera_name\": \"" << camera_to_process->get_url()
        << "\"}";

    // Send message to receiver
    df.send_message(ss.str().c_str(), ss.str().size());
}