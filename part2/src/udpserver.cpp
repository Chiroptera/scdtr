/**********************************
Copyright ACSDC/DEEC TECNICO LISBOA
alex@isr.ist.utl.pt
All rights reserved
***********************************/
#include <ctime>
#include <iostream>
#include <string>
#include <sstream>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>

#include "threadhello.h"
#include "udpserver.h"

using boost::asio::ip::udp;

udp_server::udp_server(boost::asio::io_service& io_service, int port_number, Arduino *myMicro,
                       Arduino *neighbour1Micro, std::string neighbour1Add,
                       Arduino *neighbour2Micro, std::string neighbour2Add)

    : socket_(io_service, udp::endpoint(udp::v4(), port_number)), micro_(myMicro),
      micro1_(neighbour1Micro), add1(neighbour1Add),
      micro2_(neighbour2Micro), add2(neighbour2Add)
{
   start_receive();
}

void udp_server::start_receive()
{
    socket_.async_receive_from(
                               boost::asio::buffer(recv_buffer_),
                               remote_endpoint_,
                               boost::bind(&udp_server::handle_receive, this,
                                           boost::asio::placeholders::error));
}

void udp_server::handle_receive(const boost::system::error_code& error)
{
   if (!error || error == boost::asio::error::message_size)
   {
       // get string with the message
       std::string data = std::string(recv_buffer_.begin(),recv_buffer_.end());

       // get sender address
       std::string senderAdd = remove_endpoint_.address().to_string();

       // if the sender was neighbour 1 the message is a status message
       if (senderAdd == add1){
           micro1_->set_parameters(data);
       }

       // if the sender was neighbour 2 the message is a status message
       else if (speakerAdd == add2){
           micro2_->set_parameters(data);
       }

       // else echo my status back to sender and update status
       else {
           boost::shared_ptr<std::string> message( new std::string(micro_->getString()) );
           socket_.async_send_to(boost::asio::buffer(*message), remote_endpoint_,
                                 boost::bind(&udp_server::handle_send, this, message, boost::asio::placeholders::error));

           //modify the state of the environment according to the received information
           std::string data = std::string(recv_buffer_.begin(),recv_buffer_.end());
           std::istringstream is(data);
           int val;
           is >> std::hex >> val;
           std::cout << "the value received was " << data << "which is " << val << std::endl;
           if(!is.fail())
               std::cout << "WRITING TO ARDUINO " << val << std::endl;
       }

      /*
        code for parsing matrix

       */

      start_receive();
   }
}

void udp_server::handle_send(boost::shared_ptr<std::string> msg, const boost::system::error_code& error)
{
}
