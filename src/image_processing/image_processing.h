#pragma once

#include <unordered_map>

#include "data_handler.h"
#include "data_forwarder.h"
#include "camera.h"

class image_processing: public data_handler {
    public:

        image_processing();

        ~image_processing();

        inline void process_msg(const zmq::message_t& identity, const zmq::message_t& payload) override;

    private:

        data_forwarder df;
        int received_frame_count;
        std::unordered_map<std::string, Video_Camera*> camera_url_to_camera_info_map;
};