#pragma once

#include <string>
#include <set>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

typedef websocketpp::server<websocketpp::config::asio> server;
typedef websocketpp::connection_hdl connection_hdl;

class camera_server {
    public:

        camera_server();

        ~camera_server();

        void send(const std::ostringstream& ss);

    private:
            
        server ws_server;
        std::thread server_thread;

        std::set<connection_hdl, std::owner_less<connection_hdl>> clients;
};