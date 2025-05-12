#include "camera.h"
#include "data_forwarder.h"

Video_Camera::Video_Camera(
    const std::string& url,
    double camera_lattitude,
    double camera_longitude,
    const std::vector<cv::Point2f>& imagePoints,
    const std::vector<cv::Point2f>& worldPoints,
    const double relative_y,
    const double relative_x,
    const double rotation) : 
    base_camera(
        url,
        camera_lattitude,
        camera_longitude,
        imagePoints,
        worldPoints,
        relative_y,
        relative_x,
        rotation)
{
}

Video_Camera::~Video_Camera() {
    if (camera_thread.joinable()) {
        camera_thread.join();
    }
}

void Video_Camera::start_thread() {

    this->camera_thread = std::thread([&](){ 

        // Bind to send address, may fix later
        data_forwarder df = data_forwarder(std::string("ipc:///tmp/" + std::to_string(camera_lattitude)).data(), zmq::socket_type::pub);
        df.bind();
        
        cv::VideoCapture cap(url); 
        if (!cap.isOpened()) {
            std::cerr << "Error opening video stream" << std::endl;
            return;
        }

        double fps = cap.get(cv::CAP_PROP_FPS);
        int frame_time_ms = static_cast<int>(1000.0 / fps);

        cv::Mat frame;
        int current_frame_count = 0;
        while (true) {
            auto start = std::chrono::high_resolution_clock::now();

            // Read in frame
            cap >> frame;
            if (frame.empty()) break;
            
            // Encode to passable format
            std::vector<uchar> buffer;
            cv::imencode(".png", frame, buffer);

            // Send topic message
            zmq::message_t topic(url.data(), url.size());
            df.send_message(topic, zmq::send_flags::sndmore);

            // Send image frame
            df.send_message(buffer.data(), buffer.size());
            std::cout<<++current_frame_count<<std::endl;

            // Wait for next frame if needed
            auto end = std::chrono::high_resolution_clock::now();
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

            if (elapsed_ms < frame_time_ms) {
                std::this_thread::sleep_for(std::chrono::milliseconds(frame_time_ms - elapsed_ms));
            }     
        }

        return;
    });
}

void parse_camera_json(std::vector<Video_Camera*>& list_of_cameras_from_json) {
    // Read camera information
    std::ifstream file(CAMERA_JSON_FILE_PATH);
    if (!file.is_open()) {
        std::cerr << "Failed to open JSON file." << std::endl;
        return;
    }

    // Read entire file into a string
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string json_content = buffer.str();

    // Parse JSON string into fjson object
    flatjson::fjson json(json_content.data(), json_content.size());
    
    for (int i = 0; i < json.size(); i++){
        auto elem = json.at(i);

        std::string camera_url = elem["camera_url"].to_string();
        double camera_lattitude = elem["camera_lattitude"].to_double();
        double camera_longitude = elem["camera_longitude"].to_double();
        // parse image points
        std::vector<cv::Point2f> image_points;
        auto image_point_arr = elem.at("image_points");
        for (int i = 0; i < image_point_arr.size(); i++){
            auto elem_pair = image_point_arr.at(i);
            image_points.push_back(cv::Point2f(elem_pair.at(0).to_double(), elem_pair.at(1).to_double()));
        }

        // parse world points
        std::vector<cv::Point2f> world_points;
        auto world_points_arr = elem.at("world_points");
        for (int i = 0; i < world_points_arr.size(); i++){
            auto elem_pair = world_points_arr.at(i);
            world_points.push_back(cv::Point2f(elem_pair.at(0).to_double(), elem_pair.at(1).to_double()));
        }

        double camera_y_position_wrt_0_0 = elem["camera_y_position_wrt_0_0"].to_double();
        double camera_x_position_wrt_0_0 = elem["camera_x_position_wrt_0_0"].to_double();
        double camera_rotation = elem["camera_rotation"].to_double();
        
        Video_Camera* element = new Video_Camera(
            camera_url,
            camera_lattitude,
            camera_longitude,
            image_points,
            world_points,
            camera_y_position_wrt_0_0,
            camera_x_position_wrt_0_0,
            camera_rotation
        );

        list_of_cameras_from_json.push_back(element);
    }
}