#include <iostream>
#include <boost/asio.hpp>
#include <string>
#include <boost/array.hpp>

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

int main(){
    io_service io;
    udpClient x(io);
    std::string ip="127.0.0.1";
    int port = 10001;
    std::string resposta = "";
    resposta = x.queryServer(ip,port);
    std::cout << resposta << std::endl;
}

// int main(){
//     boost::asio::io_service io;
//     udp::resolver resolver(io);
//     udp::socket socket(io);

//     std::string addr="127.0.0.1";

//     int ports[2];
//     ports[0]=10000;
//     ports[1]=10001;
//     int i = 1;
//     udp::resolver::query query(udp::v4(),addr,std::to_string(ports[i]));
//     udp::endpoint receiver = *resolver.resolve(query);

//     socket.open(udp::v4());
//     boost::array<char,1> send_buf = {{0}};

//     socket.send_to(buffer(send_buf),receiver);
//     boost::array<char,128> recv_buf;
//     udp::endpoint sender;
//     size_t len = socket.receive_from(buffer(recv_buf),sender);
//     std::cout.write(recv_buf.data(),len);

//     socket.close();

//     return 0;
// }
