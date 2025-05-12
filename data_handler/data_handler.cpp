#include "data_handler.h"
#include <iostream>
#include <fstream>

data_handler::data_handler(
        const char* receive_data_location,
        zmq::socket_type receive_data_type) {
    // Create receive data socket
    receive_data = data_forwarder(receive_data_location, receive_data_type);

    // Bind or connect based on type
    switch (receive_data_type) {
        case zmq::socket_type::router: {
            receive_data.bind();
            break;
        }
        case zmq::socket_type::sub: {
            receive_data.connect();
            // Set lower high water mark to reduce latency on receiver side
            receive_data.get_socket().set(zmq::sockopt::rcvhwm, 100);
            break;
        }
        default:
            break;
    }
}

void data_handler::start() {
    while (true) {
        zmq::message_t identity;
        zmq::message_t payload;

        receive_data.receive(identity);
        receive_data.receive(payload);

        // Get start time for processing current frame
        auto start = std::chrono::high_resolution_clock::now();

        // std::cout<<payload.to_string()<<std::endl;

        process_msg(identity, payload);

        // End statistics
        auto end = std::chrono::high_resolution_clock::now();
        auto processing_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        // std::cout << "Processing time: " << processing_time << " µs" << std::endl;
    }
}