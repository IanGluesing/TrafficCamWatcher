#pragma once

#include <zmq.hpp>

class data_forwarder {
public:

    data_forwarder() = default;

    data_forwarder(const char* dest, zmq::socket_type);

    inline void send_message(const uint8_t* buf, size_t size) {
        zmq::message_t request(buf, size);
        socket.send(request, zmq::send_flags::none);
    }

    inline void send_message(const void* buf, size_t size) {
        zmq::message_t request(buf, size);
        socket.send(request, zmq::send_flags::none);
    }

    inline void send_message(zmq::message_t& request) {
        socket.send(request, zmq::send_flags::none);
    }

    inline void send_message(zmq::message_t& request, zmq::send_flags flag) {
        socket.send(request, flag);
    }

    inline void connect() {
        socket.connect(dest);
    }

    inline void bind() {
        socket.bind(dest);
    }

    inline void receive(zmq::message_t& request) {
        auto _ = socket.recv(request, zmq::recv_flags::none);
    }

    inline void close() {
        socket.close();
    }

    inline zmq::socket_t& get_socket() {
        return socket;
    }

private:

    const char* dest;
    zmq::socket_t socket;
    zmq::context_t context;
};