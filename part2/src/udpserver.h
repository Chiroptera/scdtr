/**********************************
Copyright ACSDC/DEEC TECNICO LISBOA
alex@isr.ist.utl.pt
All rights reserved
***********************************/
#include <ctime>
#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include "threadhello.h"
#include "nodeState.h"

using boost::asio::ip::udp;

class udp_server
{
 public:

    udp_server(boost::asio::io_service& io_service, int port_number, nodeState& state);

 private:
    void start_receive();
    void handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred);
    void handle_send(boost::shared_ptr<std::string> msg, const boost::system::error_code& error);
    udp::socket socket_;
    udp::endpoint remote_endpoint_;
    boost::array<char, 128> recv_buffer_;
nodeState& state_;
};
