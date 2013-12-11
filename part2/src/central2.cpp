#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <string>
#include "threadhello.h"
#include <vector>

using namespace boost::asio;
using namespace std;
using ip::udp;


class udpClient{
public:
    udpClient(boost::asio::io_service& io)
        : _io(io),
          _resolver(io),
          _socket(io)
    {}

  std::string queryServer(std::string addr, int port, std::string message){
        udp::resolver::query query(udp::v4(),addr,std::to_string(port));
        udp::endpoint receiver = *_resolver.resolve(query);

        _socket.open(udp::v4());
        boost::array<char,1> send_buf = {{0}};

        _socket.send_to(buffer(message.c_str(),strlen(message.c_str())),receiver);
        boost::array<char,128> recv_buf;
        udp::endpoint sender;
        size_t len = _socket.receive_from(buffer(recv_buf),sender);
	_socket.close();

	//	std::cout << std::string(recv_buf.data()).substr(0,len) << std::endl;
        return (std::string(recv_buf.data()).substr(0,len));
    }

private:
    boost::asio::io_service& _io;
    udp::socket _socket;
    udp::resolver _resolver;
};

bool testMode = false;

boost::array<Arduino,8> micros;

void getBacgroungAndCoupling(udpClient& central,int* coupling,int *background, int NumberOfClients){
  //initialize 
  for (int i=0;i++;i<NumberOfclients){

    if (testMode) {
      std::cout << "querying " << "127.0.0.1" << "in port " << port[i] << endl;
      response = central.queryServer("127.0.0.1",port[i],"00");
    }
    else {
      std::cout << "querying " << addr[i] << "in port " << port[i] << endl;
      response = central.queryServer(addr[i],port[i],"00");
    }
    
    

  }

}

void updateStates(udpClient& central,std::string *addr,int *port,int NumberOfClients){
  int i=0;
  while(i<NumberOfClients){
    std::string response = "";

    if (testMode){
      std::cout << "querying " << "127.0.0.1" << "in port " << port[i] << endl;
      response = central.queryServer("127.0.0.1",port[i],"AA");
    }
    else{
      std::cout << "querying " << addr[i] << "in port " << port[i] << endl;
      response = central.queryServer(addr[i],port[i],"");
    }

    micros[i].set_parameters(response.substr(0,11));
    micros[i].print();
    i++;
  }
  return;
}



int main(int argc, char **argv)
{

  if (argc == 2){
    testMode = true;
  }
   io_service io;
   udp::resolver resolver(io);

   const int NumberOfClients = 8;

   std::string addrs[NumberOfClients+1];
   addrs[0] = "192.168.27.202";
   addrs[1] = "192.168.27.204";
   addrs[2] = "192.168.27.203";
   addrs[3] = "192.168.27.206";
   addrs[4] = "192.168.27.205";
   addrs[5] = "192.168.27.207";
   addrs[6] = "192.168.27.208";
   addrs[7] = "192.168.27.209";
   addrs[8] = "192.168.27.201"; //professor computer

   int ports[NumberOfClients];
   ports[0]=17231;
   ports[1]=17232;
   ports[2]=17233;
   ports[3]=17234;
   ports[4]=17235;
   ports[5]=17236;
   ports[6]=17237;
   ports[7]=17238;

   udpClient central(io);
   std::cout << "starting update" << endl;
   updateStates(central,addrs,ports,NumberOfClients);
   std::cout << "update finished" << endl;

   

}
