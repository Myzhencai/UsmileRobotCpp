#pragma once

#include <map>
#include <mutex>
#include <thread>
#include <vector>
#include "RequestData.h"
#include "WebSocketClient.h"

namespace c2 {

class Request {
public:
    Request(const std::string& host, const std::string& port, size_t queueSize = 100)
        : _host(host), _port(port), _queueSize(queueSize) {
        connect();
        _checkTimeoutThread = std::move(std::thread(&Request::_checkTimeout, this));
    }

    ~Request() {
        _run = false;

        _checkTimeoutThread.join();
    }

    void connect() {
        _ws = std::make_shared<WebSocketClient>(_host,
                                  _port,
                                  std::bind(&Request::onRead, this, std::placeholders::_1),
                                  std::bind(&Request::onClose, this));
        _ws->connect();

        // 等待连接完成，15s
        for (int i = 0; i < 150; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (_ws->state() != WebSocketState::connecting) {
                break;
            }
        }
    }

    void onClose() {
        std::cout << "websocket is closed" << std::endl;

        //std::thread([this]() {
        //    std::cout << "on close" << std::endl;
        //    std::this_thread::sleep_for(std::chrono::milliseconds(100));

        //    connect();
        //}).detach();
    }

    std::shared_future<Response> send(const std::string& type,
                                      const std::string& action,
                                      const json&        data,
                                      int                timeout = 30) {
        std::lock_guard<std::mutex> lock(_mutex);
        RequestData                 requestData;
        if (_requests.size() >= _queueSize) {
            requestData.error(ResponseCode::QueueFull, "queue is full");

            return requestData.future();
        }

        if (_ws->state() == WebSocketState::closed) {
            connect();
        }

        for (int i = 0; i < 300; i++) {
            if (_ws->state() == WebSocketState::connecting) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } else {
                break;
            }
        }

        if (_ws->state() != WebSocketState::connected) {
            requestData.error(ResponseCode::NetError, "websocket is not connected");

            return requestData.future();
        } else {
            _id++;

            json msg = {
                {"id",     _id   },
                {"type",   type  },
                {"action", action},
                {"data",   data  },
            };
            std::cout << "send msg: " << msg.dump(4) << std::endl;
            requestData.set(_id, timeout);
            _requests[_id] = std::move(requestData);

            _ws->Send(std::make_shared<std::string const>(msg.dump()));

            return _requests[_id].future();
        }
    }

    void onRead(std::string msg) {
        std::lock_guard<std::mutex> lock(_mutex);
        try {
            json data = json::parse(msg);
            std::cout << "recieve msg: " << data.dump(4) << std::endl;
            size_t id = data["id"];
            if (_requests.find(id) != _requests.end()) {
                _requests[id].reply(data);
                _requests.erase(id);
            } else {
                std::cout << "request is not exist by id: " << id << std::endl;
                std::cout << "message: " << msg << std::endl;
            }
        } catch (std::exception const& e) {
            std::cout << "unknown message: " << msg << std::endl;
        }
    }

private:
    void _checkTimeout() {
        while (_run) {
            _mutex.lock();

            std::vector<size_t> timeoutIds;
            for (auto& [id, requestData] : _requests) {
                if (requestData.checkTimeout()) {
                    requestData.error(ResponseCode::Timeout, "timeout");
                    timeoutIds.push_back(id);
                }
            }

            for (auto& id : timeoutIds) {
                _requests.erase(id);
            }

            _mutex.unlock();

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    std::string _host;
    std::string _port;

    std::shared_ptr<WebSocketClient> _ws;
    size_t                           _id{0};
    std::mutex                       _mutex;
    std::map<size_t, RequestData>    _requests;
    size_t                           _queueSize;
    std::thread                      _checkTimeoutThread;
    bool                             _run{true};
};

}  // namespace c2
