#include "data_forwarder.h"
#include <iostream>

data_forwarder::data_forwarder(const char* dest, zmq::socket_type type) {
    this->dest = dest;
    context = zmq::context_t(1);
    socket = zmq::socket_t(context, type);
}