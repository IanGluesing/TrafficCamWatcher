#include "camera_server.h"


camera_server::camera_server() {
    ws_server.init_asio();

    ws_server.set_open_handler([&](connection_hdl hdl) {
        std::cout << "Client connected" << std::endl;
        clients.insert(hdl);
    });

    ws_server.set_close_handler([&](connection_hdl hdl) {
        std::cout << "Client disconnected" << std::endl;
        clients.erase(hdl);
    });

    // Optional: handle messages from clients if you want
    ws_server.set_message_handler([](connection_hdl, server::message_ptr msg) {
        std::cout << "Received from browser: " << msg->get_payload() << std::endl;
    });

    ws_server.listen(8765);
    ws_server.start_accept();

    server_thread = std::thread([&]() {
        ws_server.run();
    });
}

camera_server::~camera_server() {
    ws_server.stop_listening();

    for (auto hdl : clients) {
        ws_server.close(hdl, websocketpp::close::status::going_away, "Server shutdown");
    }

    ws_server.get_io_service().stop();

    if (server_thread.joinable()) {
        server_thread.join();
    }
}

void camera_server::send(const std::ostringstream& ss) {
    for (auto hdl : clients) {
        ws_server.send(hdl, ss.str(), websocketpp::frame::opcode::text);
    }
}
