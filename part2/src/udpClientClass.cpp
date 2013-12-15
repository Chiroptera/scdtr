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


udpClient::udpClient(boost::asio::io_service& io, std::string addr, int port)
    : _io(io),
      _resolver(io),
      _socket(io),
      _addr(addr),
      _port(port)
{}

void udpClient::echo(std::string msg){
    udp::resolver::query query(udp::v4(),_addr,std::to_string(_port));
    udp::endpoint receiver = *_resolver.resolve(query);

    _socket.open(udp::v4());

    boost::shared_ptr<std::string> message(new std::string(msg));
    _socket.send_to(boost::asio::buffer(*message), receiver);

    _socket.close();
}
