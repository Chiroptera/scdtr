#include <iostream>
#include <boost/asio.hpp>
#include <string>
#include <boost/array.hpp>
#include <thread>

using namespace boost::asio;
using ip::udp;
using namespace std;


#define Lmin 	300	// Lum for empty desk
#define Lmax 	600	// Lum for desk occupied


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

int main(int argc, char** argv<){
  if (argc != 4){
    cerr << "client myport neighbor1 neighbor2";
    return -1;
  }
  // base porto 17000
  int ports[3] = {atoi(argv[1]),atoi(argv[2]),atoi(argv[3])};
  int myPos = ports[0]-17000;
  int posL = (myPos == 0) ? 7 : myPos - 1;
  int posR = (myPos == 7) ? 0 : myPos + 1;
  

  io_service io;
  udpClient x(io);
  std::string ip="127.0.0.1";
  
  int presence=1;
  int E[8]={1000,100,100,100,100,100,100,100};
  int b=100;
  
  int desiredLux = (presence = 1) ? Lmin : Lmax;
  int led=0;
    int neighL,neighR;
  for(;;){
    neighL=atoi(x.queryServer(ip,ports[1]));
    neighR=atoi(x.queryServer(ip,ports[2]));

    led = desiredLux - neighL*E[posL] - neighR*E[posR];
    led = led / E[myPos];

    std::cout << resposta << std::endl;
  }

}
