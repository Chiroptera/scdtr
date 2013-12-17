#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>
#include <thread>
#include <deque>
#include <string>
#include <ctype.h>
//#include "threadhello.h"
#include <vector>


#include <sstream>
#include <iostream>
#include <boost/lexical_cast.hpp>

//#include "udpClientClass.h"

using namespace boost::asio;
using ip::udp;
using namespace std;
using boost::asio::ip::tcp;

/*

     GLOBAL VARIABLES

*/


/*

     FUNCTIONS

*/

// std::string getData(std::string addr, int port, udp::socket& _socket, udp::resolver& _resolver){
// std::cout << "Querying " << addr << " in port " << port << std::endl;
// udp::resolver::query query(udp::v4(),addr,std::to_string(port));
// udp::endpoint receiver = *_resolver.resolve(query);

// _socket.open(udp::v4());
// //boost::array<char,1> send_buf = {{0}};
//  char buf='';
// _socket.send_to(buffer(send_buf),receiver);
// boost::array<char,128> recv_buf;
// udp::endpoint sender;
// size_t len = _socket.receive_from(buffer(recv_buf),sender);
// //std::cout.write(recv_buf.data(),len);
// return (std::string(recv_buf.data()));

// }

int main(){

    const int NumberOfClients = 2;

    //const udpClient *clients[NumberOfClients];

    int ports[NumberOfClients];
    ports[0]=10000;
    ports[1]=10001;

    std::string addrs[NumberOfClients];
    addrs[0]="127.0.0.1";
    addrs[0]="127.0.0.1";

//    std::vector<udpClient> clients;

    // for (int i=0;i++;i<NumberOfClients){
    //     //    clients[i]=&udpClient(io);

    //     clients.push_back(udpClient(io));
    // }

    boost::asio::io_service io;
    udp::resolver resolver(io);
    udp::socket socket(io);


    for(int i=0;i<2;i++){
        //        x.queryServer("127.0.0.1","10000");
std::cout << "Querying " << addrs[i] << " in port " << ports[i] << std::endl;
        udp::resolver::query query(udp::v4(),addrs[i],std::to_string(ports[i]));
        udp::endpoint receiver = *resolver.resolve(query);

        socket.open(udp::v4());
        boost::array<char,1> send_buf = {{0}};
        socket.send_to(buffer(send_buf),receiver);

        boost::array<char,128> recv_buf;
        udp::endpoint sender;
        size_t len = socket.receive_from(buffer(recv_buf),sender);
        std::cout.write(recv_buf,len);

        socket.close();

    }
    return 0;
}
