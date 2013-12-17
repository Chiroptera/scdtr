#include <iostream>
#include <boost/asio.hpp>

using namespace boost::asio;
using ip::udp;

int main(){
    io_service io;
    udp::socket socket(io,udp::endpoint(udp::v4(),10001));
    for(;;){
        boost::array<char,1> recv;
        udp::endpoint client;
        boost::system::error_code err;
        socket.receive_from(buffer(recv),client,0,err);

        if(err && err != error::message_size){
            std::cout << "Error" << std::endl;
            return -1;
        }

        boost::system::error_code ignored;
        socket.send_to(buffer("Hello\n"),client,0,ignored);
    }
}
