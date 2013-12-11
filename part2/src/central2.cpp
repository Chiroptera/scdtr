#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <string>
#include "threadhello.h"
#include <vector>
#include <math.h>

using namespace boost::asio;
using namespace std;
using ip::udp;


class udpClient{
public:
  udpClient(boost::asio::io_service& io, std::string addr, int port)
        : _io(io),
          _resolver(io),
          _socket(io),
	  _addr(addr),
	  _port(port)
    {}

  std::string queryServer(std::string message){
        udp::resolver::query query(udp::v4(),_addr,std::to_string(_port));
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
  std::string _addr;
  int _port;
};


bool testMode = false;
const int NumberOfClients = 8;
boost::array<Arduino,8> micros;
std::vector<udpClient*> clients;


void updateStates(){
  int i=0;
  while(i<NumberOfClients){
    std::string response = "";

    std::cout << "querying in client " << i << endl;
    response = clients[i]->queryServer("");
    micros[i].set_parameters(response.substr(0,11));
    micros[i].print();
    i++;
  }
  return;
}

void getBacgroungAndCoupling(double coupling[][NumberOfClients],double background[NumberOfClients]){

  // initialize all workstations with LED at 0% PWM
  for (int i=0;i<NumberOfClients;i++){
      clients[i]->queryServer("00");
  }

  // update for background data
  updateStates();

  double aux;

  // get background data
  std::cout << "\n\n\n\n doing background \n\n\n";
  for (int i=0;i<NumberOfClients;i++){
    aux = 198 - micros[i].getLDR();
    aux = aux / 33;
    background[i] = pow(10.00,aux);
  }


  // coupling matrix
  std::cout << "\n\n\n\n doing coupling \n\n\n";
  for (int i=0;i<NumberOfClients;i++){

    // change LED i to 01
    clients[i]->queryServer("FF");

    // update all info
    updateStates();

    // update coupling matrix
    for (int j=0;j<NumberOfClients; j++){
      std::cout << "effect of micro " << i << " in micro " << j << " is " << micros[j].getLDR() << std::endl;
      aux = 198 - micros[j].getLDR();
      aux = aux / 33;
      coupling[i][j] = pow(10.00,aux);
    }

    // restore LED i to 00
    clients[i]->queryServer("00");
  }

}

int main(int argc, char **argv)
{

  if (argc == 2){
    testMode = true;
  }

   io_service io;
   udp::resolver resolver(io);

   double coupling[NumberOfClients][NumberOfClients]={{0}};
   double background[NumberOfClients]{0};

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



   for (int i=0;i<NumberOfClients;i++){
     if (testMode) clients.push_back(new udpClient(io,"127.0.0.1",ports[i]));
     else  clients.push_back(new udpClient(io,addrs[i],ports[i]));
   }


   //   udpClient central(io);
   std::cout << "starting update" << endl;
   updateStates();
   std::cout << "update finished" << endl;

   getBacgroungAndCoupling(coupling,background);

   // print background matrix
   std::cout << "\n\nBACKGROUND MATRIX\n\n";
   for (int i=0; i<NumberOfClients;i++){
     std::cout << background[i] << ",";
   }
   std::cout << std::endl;

   // print coupling matrix
   std::cout << "\n\nCOUPLING MATRIX\n\n";
   for (int i=0; i<NumberOfClients;i++){
     std::cout << "\t";
     for (int j=0; j<NumberOfClients;j++){
       std::cout << coupling[i][j] << ",";
     }
     std::cout << std::endl;
   }
   
}
