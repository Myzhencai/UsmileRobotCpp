#include "DeviceManager.hpp"

DeviceManager::DeviceManager() {
    // Initialize WebSocket clients
    robotWs = std::make_unique<Client>();
    cameraWs = std::make_unique<Client>();
    forceWs = std::make_unique<Client>();

    // Set up ASIO
    robotWs->set_access_channels(websocketpp::log::alevel::all);
    robotWs->clear_access_channels(websocketpp::log::alevel::frame_payload);
    robotWs->init_asio();

    cameraWs->set_access_channels(websocketpp::log::alevel::all);
    cameraWs->clear_access_channels(websocketpp::log::alevel::frame_payload);
    cameraWs->init_asio();

    forceWs->set_access_channels(websocketpp::log::alevel::all);
    forceWs->clear_access_channels(websocketpp::log::alevel::frame_payload);
    forceWs->init_asio();
}

DeviceManager::~DeviceManager() {
    disconnectAll();
}

bool DeviceManager::connectAll() {
    try {
        // Connect to robot
        websocketpp::lib::error_code ec;
        Client::connection_ptr con = robotWs->get_connection("ws://robot_ip:port", ec);
        if (ec) {
            std::cout << "[设备连接错误] Robot: " << ec.message() << std::endl;
            return false;
        }
        robotWs->connect(con);
        robotConnection = con->get_handle();

        // Connect to camera
        con = cameraWs->get_connection("ws://camera_ip:port", ec);
        if (ec) {
            std::cout << "[设备连接错误] Camera: " << ec.message() << std::endl;
            return false;
        }
        cameraWs->connect(con);
        cameraConnection = con->get_handle();

        // Connect to force sensor
        con = forceWs->get_connection("ws://force_sensor_ip:port", ec);
        if (ec) {
            std::cout << "[设备连接错误] Force Sensor: " << ec.message() << std::endl;
            return false;
        }
        forceWs->connect(con);
        forceConnection = con->get_handle();

        return true;
    } catch (const std::exception& e) {
        std::cout << "[设备连接错误] " << e.what() << std::endl;
        return false;
    }
}

bool DeviceManager::checkStatus() {
    try {
        // Send ping to robot
        robotWs->send(robotConnection, "ping", websocketpp::frame::opcode::text);
        return true;
    } catch (const std::exception& e) {
        std::cout << "[状态检查错误] " << e.what() << std::endl;
        return false;
    }
}

void DeviceManager::disconnectAll() {
    try {
        if (robotWs) {
            robotWs->close(robotConnection, websocketpp::close::status::normal, "Disconnecting");
        }
        if (cameraWs) {
            cameraWs->close(cameraConnection, websocketpp::close::status::normal, "Disconnecting");
        }
        if (forceWs) {
            forceWs->close(forceConnection, websocketpp::close::status::normal, "Disconnecting");
        }
    } catch (const std::exception& e) {
        std::cout << "[断开连接错误] " << e.what() << std::endl;
    }
} 