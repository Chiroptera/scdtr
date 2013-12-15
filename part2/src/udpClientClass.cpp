#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <thread>
#include <string>
#include <ctype.h>

#include <sstream>
#include <iostream>

#include "udpClientClass.h"

using namespace boost::asio;
using ip::udp;
using namespace std;


class udpClient{
public:
    udpClient(boost::asio::io_service& io)
        : _io(io),
          _resolver(io),
          _socket(io)
    {}

    std::string queryServer(std::string addr, int port){
        udp::resolver::query query(udp::v4(),addr,std::to_string(port));
        udp::endpoint receiver = *_resolver.resolve(query);

        _socket.open(udp::v4());
        boost::array<char,1> send_buf = {{0}};

        _socket.send_to(buffer(send_buf),receiver);
        boost::array<char,128> recv_buf;
        udp::endpoint sender;
        size_t len = _socket.receive_from(buffer(recv_buf),sender);
        //std::cout.write(recv_buf.data(),len);
        return (std::string(recv_buf.data()));
    }

private:
    boost::asio::io_service& _io;
    udp::socket _socket;
    udp::resolver _resolver;
};
