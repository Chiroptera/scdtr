#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <thread>
#include <string>
#include <ctype.h>

#include <sstream>
#include <iostream>



class udpClient{
public:
    udpClient(boost::asio::io_service& io, std::string addr, int port);
    void echo(std::string msg);

 private:
    boost::asio::io_service& _io;
    boost::asio::ip::udp::resolver _resolver;
    boost::asio::ip::udp::socket _socket;
    std::string _addr;
    int _port;
};
