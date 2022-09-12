#ifndef DATAGRAMSERVER_H
#define DATAGRAMSERVER_H

#include <cstdlib>
#include <thread>
#include <atomic>
#include <iostream>
#include <asio.hpp>

/// Synchronous UDP server with timeout. NOT thread safe.
class DatagramServer
{

public:

    DatagramServer()
        : socket_(NULL)
    {}

    bool open(const std::string& hostname, const int& localPort, const int& remotePort)
    {
        try
        {
            socket_ = new asio::ip::udp::socket(io_service_, asio::ip::udp::endpoint(asio::ip::udp::v4(), localPort));
            std::cout <<"Socket Avaliable" <<socket_->available() << std::endl;
            asio::ip::udp::resolver resolver(io_service_);
            std::stringstream ss; ss << remotePort;
            endpoint_ = *resolver.resolve({asio::ip::udp::v4(), hostname, ss.str()});
            received_.store(false);
        }
        catch (std::exception& e)
        {
            std::cerr << "Exception while opening DatagramServer: " << e.what() << std::endl;
            return false;
        }
        return true;
    }

    bool close()
    {
        bool closeOK = true;
        if (socket_ != NULL)
        {
            try
            {
                if (socket_->is_open())
                    socket_->close();
            }
            catch (std::exception& e)
            {
                std::cerr << "Exception while closing DatagramServer: " << e.what() << std::endl;
                closeOK = false;
            }
            delete socket_;
            socket_ = NULL;
        }
        return closeOK;
    }

    ~DatagramServer()
    {
        close();
    }

    bool receive(void * data, const unsigned &size, const double &timeOut_ms)
    {
        std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> t = std::chrono::high_resolution_clock::now() - start;

        received_.store(false);
        socket_->async_receive_from(asio::buffer(data, size), endpoint_,

        [this](std::error_code ec, std::size_t bytes_recvd)
        {
            if (!ec && bytes_recvd > 0)
                this->received_.store(true);
            else
                std::cerr << "DatagramServer::receive() exception: " << ec.message() << std::endl;
        });

        while (t.count() < timeOut_ms && received_.load() == false)
        {
            io_service_.poll();
            io_service_.reset();
            t = std::chrono::high_resolution_clock::now() - start;
        }
        return received_.load();
    }

    void send(void * data, const unsigned &size)
    {
        socket_->send_to(asio::buffer(data, size), endpoint_);
    }

private:

    asio::io_service io_service_;
    asio::ip::udp::socket *socket_;
    asio::ip::udp::endpoint endpoint_;
    std::atomic<bool> received_;

    DatagramServer(const DatagramServer &) = delete;
    DatagramServer& operator = (const DatagramServer &) = delete;
};

#endif // DATAGRAMSERVER_H
