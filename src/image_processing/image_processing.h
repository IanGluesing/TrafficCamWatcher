#pragma once

#include "data_handler.h"
#include "data_forwarder.h"

class image_processing: public data_handler {
    public:

        image_processing();

        ~image_processing();

        inline void process_msg(const zmq::message_t& identity, const zmq::message_t& payload) override;

    private:

        data_forwarder df;
};