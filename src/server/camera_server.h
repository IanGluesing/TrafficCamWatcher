#pragma once

#include <string>
#include <set>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include "data_handler.h"

typedef websocketpp::server<websocketpp::config::asio> server;
typedef websocketpp::connection_hdl connection_hdl;

class camera_server: public data_handler {
    public:

        camera_server();

        ~camera_server();

        void send(const std::ostringstream& ss);

        inline void process_msg(const flatjson::fjson& json_obj) override {
            for (auto hdl : clients) {
                ws_server.send(hdl, json_obj.dump(), websocketpp::frame::opcode::text);
            }
        }

    private:
            
        server ws_server;
        std::thread server_thread;

        std::set<connection_hdl, std::owner_less<connection_hdl>> clients;
};