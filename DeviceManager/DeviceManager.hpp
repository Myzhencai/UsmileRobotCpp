#pragma once

#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_client.hpp>
#include <memory>
#include <string>
#include <iostream>

class DeviceManager {
public:
    DeviceManager();
    ~DeviceManager();

    bool connectAll();
    bool checkStatus();
    void disconnectAll();

private:
    using Client = websocketpp::client<websocketpp::config::asio_tls_client>;
    using ClientPtr = std::unique_ptr<Client>;
    using ConnectionHandle = websocketpp::connection_hdl;

    ClientPtr robotWs;
    ClientPtr cameraWs;
    ClientPtr forceWs;

    ConnectionHandle robotConnection;
    ConnectionHandle cameraConnection;
    ConnectionHandle forceConnection;
}; 