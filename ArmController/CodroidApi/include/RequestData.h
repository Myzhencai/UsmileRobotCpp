#pragma once

#include <chrono>
#include <future>
#include <utility>
#include "Define.h"

namespace c2 {

class RequestData {
public:
    RequestData() {
        _sharedFuture = _promise.get_future();
    }

    void set(size_t id, int timeout) {
        _id      = id;
        _timeout = timeout;

        _startTime =
            std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch())
                .count();
    }

    bool checkTimeout() const {
        auto now = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch())
                       .count();
        
        if (now > _startTime + _timeout) {
            std::cout << "id: " << _id << " timeout" << std::endl;
            std::cout << "_startTime: " << _startTime << std::endl;
            std::cout << "_timeout: " << _timeout << std::endl;
            std::cout << "now: " << now << std::endl;
            return true;
        }
     
        return false;
    }

    void error(const ResponseCode code, const std::string& msg) {
        std::cout << "error: " << msg << std::endl;
        _promise.set_value({code, msg, json::array()});
    }

    void reply(const json& data) {
        std::cout << "reply: " << data.dump(4) << std::endl;
        int code = data["code"].get<int>();
        if (code == 200) {
            _promise.set_value({ResponseCode::OK, "ok", data["data"]});
        } else {
            error(ResponseCode::RequestFailed, std::string("request failed by code: ").append(std::to_string(code)));
        }
    }

    std::shared_future<Response> future() const {
        return _sharedFuture;
    }

private:
    int                           _timeout{0};
    size_t                        _startTime{0};
    size_t                        _id{0};
    std::promise<Response>        _promise;
    std::shared_future<Response>  _sharedFuture;
    std::function<void(Response)> _callBack;
};

}  // namespace c2
