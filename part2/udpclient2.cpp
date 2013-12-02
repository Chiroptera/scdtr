#include <iostream>
#include <boost/asio.hpp>
#include <string>

using namespace boost::asio;
using ip::udp;


class udpClient{
public:
    udpClient(boost::asio::io_service& io)
        : _io(io),
          resolver(io)
    {}

    void queryServer(std::string addr, std::string port){
        udp::resolver::query query(udp::v4(),addr,port);
        udp::endpoint receiver = *resolver.resolve(query);

        socket.open(udp::v4());
        boost::array<char,1> send_buf = {{0}};

        socket.send_to(buffer(send_buf),receiver);
        boost::array<char,128> recv_buf;
        udp::endpoint sender;
        size_t len = socket.receive_from(buffer(recv_buf),sender);
        std::cout.write(recv_buf.data(),len);
    }

private:
    boost::asio::io_service& _io;
    udp::socket _socket;
    udp::resolver resolver;
};

int main(){
    io_service io;
    udpClient x(io);
    for(;;){
        udpClient.queryServer("127.0.0.1","10000");
    }
}
