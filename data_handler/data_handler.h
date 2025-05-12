#pragma once

#include "data_forwarder.h"
#include "flatjson.hpp"

class data_handler {
public:

    data_handler(
        const char* receive_data_location,
        zmq::socket_type receive_data_type);

    virtual inline void process_msg(const zmq::message_t& identity, const zmq::message_t& payload) = 0;

    virtual void start();

protected:

    data_forwarder receive_data;
};