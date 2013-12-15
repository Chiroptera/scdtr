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

#include "nodeState.h"
#include "threadhello.h"
#include "udpserver.h"
#include "udpClientClass.h"


#ifdef POSIX
#include <termios.h>
#endif


using namespace boost::asio;

using namespace std;


void taskEchoState(udpClient *client,nodeState *state,int mode)
{
if (mode == 1) return;
for(;;)
    {
client->echo(state->micro_.getString());
        client->echo(state->getOccString());
        usleep(100000);
        }

        }


int main(int argc,char** argv){

int myPort,neighbour1Port,neighbour2Port;
    std::string neighbour1Add,neighbour2Add;
    std::string neighborAddFirst, neighborAddSecond, token = ":";
        int mode =0;
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



    boost::asio::io_service io_service;

        nodeState state(myPort,neighbour1Add,neighbour2Add);

        udp_server server(io_service, myPort,state);

    std::thread t(boost::bind(&boost::asio::io_service::run, &io_service)); // thread for running the io service

udpClient cNeighbour1(io_service,neighbour1Add,neighbour1Port);
udpClient cNeighbour2(io_service,neighbour2Add,neighbour2Port);


    std::thread tNeighbour1(boost::bind(taskEchoState,&cNeighbour1,&state,mode));
        std::thread tNeighbour2(boost::bind(taskEchoState,&cNeighbour2,&state,mode));


    for(int i=0;;i++)
        {
            i = (i==10) ? 0 : i;
            state.micro_.set_parameters(std::string(std::to_string(i) + std::to_string(i) + "11223344 " ));
            cout << "\n\nMY ARDUINO VALUES" << endl;
            state.micro_.print();
            cout << "\n\nNEIGHBOUR #1 ARDUINO" << endl;
            state.micro1_.print();
            cout << "\n\nNEIGHBOUR #2 ARDUINO" << endl;
            state.micro2_.print();

                cout << "\n\nCOUPLING" << endl;
                    for (int i=0;i<8;i++)
                        {
                        for (int j=0;j<8;j++)
                                 {
                                 cout << state.coupling_[i][j] << ",";
                                 }
                                 cout << endl;
                        }

                        cout << "\n\nBACKGROUND" << endl;
                            for (int i=0;i<8;i++){cout << state.background_[i] << ",";}
            cout << endl;
            usleep(2000000);

            }

            t.join();
                tNeighbour1.join();
                    tNeighbour2.join();


            }
