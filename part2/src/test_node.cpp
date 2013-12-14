#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <boost/lexical_cast.hpp>

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
        neighbour2Add = std::string(argv[2]);
        neighbour2Port = atoi(neighbour2Add.substr(neighbour2Add.find(token)+1,5).c_str());
        neighbour2Add = neighbour2Add.substr(0,neighbour2Add.find(token));

        cout << "My port is " << myPort << endl;
        std::cout << "Neighbour 1\nPort:\t" << neighbour1Port << "\tAddress:\t" << neighbour1Add << std::endl;
        std::cout << "Neighbour 2\nPort:\t" << neighbour2Port << "\tAddress:\t" << neighbour2Add << std::endl;

    }

    boost::array<Arduino,3> micros;
    micros[0].set_parameters(std::string("AA11223344 "));

    boost::asio::io_service io_service;
    udp_server server(io_service, myPort,&micros[0],&micros[1],neighbour1Add,&micros[2],neighbour2Add);

}
