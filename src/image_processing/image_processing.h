#pragma once

#include <unordered_map>

#include "data_handler.h"
#include "data_forwarder.h"
#include "camera.h"

class image_processing: public data_handler {
    public:

        image_processing(Video_Camera* camera_to_process);

        ~image_processing();

        virtual void start() override;

        inline void process_msg(const zmq::message_t& identity, const zmq::message_t& payload) override;

    private:

        data_forwarder df;
        Video_Camera* camera_to_process;
        int received_frame_count;
};