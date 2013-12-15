#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <unistd.h>

#include <boost/array.hpp>
#include <vector>
#include <deque>

#include <thread>

#include <string>
#include <ctype.h>

#include <sstream>
#include <iostream>


#include "threadhello.h"
#include "udpserver.h"

#ifdef POSIX
#include <termios.h>
#endif


using namespace boost::asio;
using ip::udp;
using namespace std;

class udpClient{
public:
    udpClient(boost::asio::io_service& io, std::string addr, int port)
        : _io(io),
          _resolver(io),
          _socket(io),
	  _addr(addr),
	  _port(port)
    {}

    void echo(std::string msg){
        udp::resolver::query query(udp::v4(),_addr,std::to_string(_port));
        udp::endpoint receiver = *_resolver.resolve(query);

        _socket.open(udp::v4());

boost::shared_ptr<std::string> message(new std::string(msg));
_socket.send_to(boost::asio::buffer(*message), receiver);

	_socket.close();
    }

private:
    boost::asio::io_service& _io;
    udp::resolver _resolver;
    udp::socket _socket;
    std::string _addr;
    int _port;
};

 void taskEchoState(udpClient *client,Arduino *micro)
 {
     for(;;)
         {
             client->echo(micro->getString());
             usleep(100000);
             }

             }


int main(int argc,char** argv){

int myPort,neighbour1Port,neighbour2Port;
    std::string neighbour1Add,neighbour2Add;
    std::string neighborAddFirst, neighborAddSecond, Add1, Add2, port1,port2,token = ":";


    if (argc!=4)
    {
        cerr << "Usage: testnode <my port> <neighbour 1 add:port> <neighbour 1 add:port> \n" << endl;
        return -1;
    }
    else if (argc == 4){
        // get my port from argument 1
        myPort = atoi(argv[1]);

        // get neighbour1's address and port
        neighbour1Add = std::string(argv[2]);
        neighbour1Port = atoi(neighbour1Add.substr(neighbour1Add.find(token)+1,5).c_str());
        neighbour1Add = neighbour1Add.substr(0,neighbour1Add.find(token));

        // get neighbour1's address and port
        neighbour2Add = std::string(argv[3]);
        neighbour2Port = atoi(neighbour2Add.substr(neighbour2Add.find(token)+1,5).c_str());
        neighbour2Add = neighbour2Add.substr(0,neighbour2Add.find(token));

        cout << "My port is " << myPort << endl;
        std::cout << "Neighbour 1\nPort:\t" << neighbour1Port << "\tAddress:\t" << neighbour1Add << std::endl;
        std::cout << "Neighbour 2\nPort:\t" << neighbour2Port << "\tAddress:\t" << neighbour2Add << std::endl;

    }

    boost::array<Arduino,3> micros;

    boost::asio::io_service io_service;
    udp_server server(io_service, myPort,&micros[0],&micros[1],neighbour1Add,&micros[2],neighbour2Add);

    std::thread t(boost::bind(&boost::asio::io_service::run, &io_service)); // thread for running the io service

udpClient cNeighbour1(io_service,neighbour1Add,neighbour1Port);
udpClient cNeighbour2(io_service,neighbour2Add,neighbour2Port);

std::thread tNeighbour1(boost::bind(taskEchoState,&cNeighbour1,&micros[0]));
std::thread tNeighbour2(boost::bind(taskEchoState,&cNeighbour2,&micros[0]));


    for(int i=0;;i++)
        {
            i = (i==10) ? 0 : i;
            micros[0].set_parameters(std::string(std::to_string(i) + std::to_string(i) + "11223344 " ));
            cout << "\n\nMY ARDUINO VALUES" << endl;
            micros[0].print();
            cout << "\n\nNEIGHBOUR #1 ARDUINO" << endl;
            micros[1].print();
            cout << "\n\nNEIGHBOUR #2 ARDUINO" << endl;
            micros[2].print();
            usleep(1000000);

            }
            t.join();
            tNeighbour1.join();
            tNeighbour2.join();
            }
