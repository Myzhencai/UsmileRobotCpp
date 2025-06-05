#pragma once

#include <boost/asio/strand.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <queue>

namespace beast     = boost::beast;          // from <boost/beast.hpp>
namespace http      = beast::http;           // from <boost/beast/http.hpp>
namespace websocket = beast::websocket;      // from <boost/beast/websocket.hpp>
namespace net       = boost::asio;           // from <boost/asio.hpp>
using tcp           = boost::asio::ip::tcp;  // from <boost/asio/ip/tcp.hpp>

namespace c2 {

enum class WebSocketState : int {
    connecting = 1,
    connected,
    closing,
    closed,
};

class WebSocketClient: public std::enable_shared_from_this<WebSocketClient> {
public:
    
    WebSocketClient(const std::string&               host,
                    const std::string&               port,
                    std::function<void(std::string)> onRead,
                    std::function<void()> onClose)
        : _host(host),
          _port(port),
          _resolver(net::make_strand(_ioc)),
          _ws(net::make_strand(_ioc)),
          _receive(onRead),
          _onClosed(onClose) {
        //connect();
    }

    ~WebSocketClient() {
        _ioc.stop();
    }

    void connect() {
        _resolver.async_resolve(_host,
                                _port,
                                beast::bind_front_handler(&WebSocketClient::_onResolve, shared_from_this()));
        std::cout << "websocket resolve" << std::endl;
        _state = WebSocketState::connecting;

        std::thread([this]() {
            //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            std::cout << "_ioc run" << std::endl;
            _ioc.run();

            std::cout << "_ioc thread quit" << std::endl;
        }).detach();
    }

    void Send(std::shared_ptr<std::string const> const& msg) {
        net::post(_ws.get_executor(), beast::bind_front_handler(&WebSocketClient::_onSend, shared_from_this(), msg));

        //std::cout << "send msg: " << msg << std::endl;
        //_ws.async_write(net::buffer(msg), beast::bind_front_handler(&WebSocketClient::_onWrite, this));
    }

    void close() {
        // Close the WebSocket connection
        _state = WebSocketState::closing;
        _ws.async_close(websocket::close_code::normal,
                        beast::bind_front_handler(&WebSocketClient::_onClose, shared_from_this()));
    }

    WebSocketState state() const {
        return _state;
    }

private:
    void _onResolve(beast::error_code ec, tcp::resolver::results_type results) {
        if (ec) {
            std::cerr << "resolve failed: " << ec.message() << std::endl;
            _state = WebSocketState::closed;
            return;
        }

        // Set the timeout for the operation
        beast::get_lowest_layer(_ws).expires_after(std::chrono::seconds(30));

        // Make the connection on the IP address we get from a lookup
        beast::get_lowest_layer(_ws).async_connect(
            results,
            beast::bind_front_handler(&WebSocketClient::_onConnect, shared_from_this()));
        std::cout << "websocket connect" << std::endl;
    }

    void _onConnect(beast::error_code ec, tcp::resolver::results_type::endpoint_type ep) {
        if (ec) {
            std::cerr << "connect failed: " << ec.message() << std::endl;
            _state = WebSocketState::closed;
            return;
        }

        // Turn off the timeout on the tcp_stream, because
        // the websocket stream has its own timeout system.
        beast::get_lowest_layer(_ws).expires_never();

        // Set suggested timeout settings for the websocket
        _ws.set_option(websocket::stream_base::timeout::suggested(beast::role_type::client));

        // Set a decorator to change the User-Agent of the handshake
        _ws.set_option(websocket::stream_base::decorator([](websocket::request_type& req) {
            req.set(http::field::user_agent, std::string(BOOST_BEAST_VERSION_STRING) + " websocket-client-async");
        }));

        // Update the host_ string. This will provide the value of the
        // Host HTTP header during the WebSocket handshake.
        // See https://tools.ietf.org/html/rfc7230#section-5.4
        std::string host = _host + ':' + std::to_string(ep.port());

        // Perform the websocket handshake
        _ws.async_handshake(_host, "/", beast::bind_front_handler(&WebSocketClient::_onHandshake, shared_from_this()));
        std::cout << "websocket handshake" << std::endl;
    }

    void _onHandshake(beast::error_code ec) {
        if (ec) {
            std::cerr << "handshake failed: " << ec.message() << std::endl;
            _state = WebSocketState::closed;
            return;
        }
        std::cout << "websocket is connected" << std::endl;
        _state = WebSocketState::connected;

        _buffer.clear();
        // Read a message into our buffer
        _ws.async_read(_buffer, beast::bind_front_handler(&WebSocketClient::_onRead, shared_from_this()));
    }

    void _onSend(std::shared_ptr<std::string const> const& msg) {
        _queue.push(msg);

        if (_queue.size() > 1) {
            return;
        }

        _ws.async_write(net::buffer(*_queue.front()),
                        beast::bind_front_handler(&WebSocketClient::_onWrite, shared_from_this()));
    }

    void _onWrite(beast::error_code ec, std::size_t bytes_transferred) {
        boost::ignore_unused(bytes_transferred);

        if (ec) {
            if (_ws.is_open()) {
                std::cerr << "write failed: " << ec.message() << std::endl;
            } else {
                std::cout << "_onWrite: websocket is closed" << std::endl;
                _state = WebSocketState::closed;
                _onClosed();
                return;
            }
        }

        _queue.pop();

        if (!_queue.empty()) {
            _ws.async_write(net::buffer(*_queue.front()),
                            beast::bind_front_handler(&WebSocketClient::_onWrite, shared_from_this()));
        }
    }

    void _onRead(beast::error_code ec, std::size_t bytes_transferred) {
        boost::ignore_unused(bytes_transferred);

        if (ec) {
            if (_ws.is_open()) {
                std::cerr << "read failed: " << ec.message() << " errorCode: " << ec << std::endl;
            } else {
                std::cout << "_onRead: websocket is closed" << std::endl;
                _state = WebSocketState::closed;
                _onClosed();
                return;
            }
        } else {
            // std::cout << "read msg: " << beast::buffers_to_string(_buffer.data()) << std::endl;
            _receive(beast::buffers_to_string(_buffer.data()));
        }
        
        // Read a message into our buffer
        _buffer.consume(_buffer.size());
        _ws.async_read(_buffer, beast::bind_front_handler(&WebSocketClient::_onRead, shared_from_this()));
    }

    void _onClose(beast::error_code ec) {
        if (ec) {
            std::cerr << "close failed: " << ec.message() << std::endl;
            return;
        }

        // If we get here then the connection is closed gracefully
        _state = WebSocketState::closed;
    }

    std::string _host;
    std::string _port;

    
    net::io_context                            _ioc;
    tcp::resolver                        _resolver;
    websocket::stream<beast::tcp_stream> _ws;
    beast::flat_buffer                   _buffer;
    std::function<void(std::string)>     _receive;
    std::function<void()>                _onClosed;
    WebSocketState                       _state{WebSocketState::closed};
    std::queue<std::shared_ptr<std::string const>> _queue;
};

}  // namespace c2
