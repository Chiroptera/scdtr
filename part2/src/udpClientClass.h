#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <thread>
#include <string>
#include <ctype.h>

#include <sstream>
#include <iostream>



class udpClient{
public:
    udpClient(boost::asio::io_service& io);
    std::string queryServer(std::string addr, int port);

private:
    boost::asio::io_service& _io;
    boost::asio::ip::udp::socket _socket;
    boost::asio::ip::udp::resolver _resolver;
};
