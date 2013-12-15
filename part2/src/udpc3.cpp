#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>

using namespace boost::asio;
using ip::udp;

int main(){
    io_service io;
    udp::resolver resolver(io);
    udp::resolver::query query(udp::v4(),"192.168.56.101","17231");
    udp::endpoint receiver = *resolver.resolve(query);
    udp::socket socket(io);

    socket.open(udp::v4());
    boost::array<char,1> send_buf = {{0}};

    socket.send_to(buffer(send_buf),receiver);
    boost::array<char,128> recv_buf;
    udp::endpoint sender;
    size_t len = socket.receive_from(buffer(recv_buf),sender);
    std::cout.write(recv_buf.data(),len);
}
