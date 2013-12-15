#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <boost/array.hpp>
#include <vector>
#include <deque>

#include <thread>

#include <string>
#include <ctype.h>

#include <sstream>
#include <iostream>

#include <boost/lexical_cast.hpp>

#include "nodeState.h"
#include "threadhello.h"
#include "udpserver.h"
#include "udpClientClass.h"

#ifdef POSIX
#include <termios.h>
#endif


using namespace boost::asio;
using namespace std;

class minicom_client
{
public:
    minicom_client(boost::asio::io_service& io_service, unsigned int baud, const string& device, nodeState& state)
        : active_(true),
          io_service_(io_service),
          serialPort(io_service, device),
          state_(state)
    {
        if (!serialPort.is_open())
        {
            cerr << "Failed to open serial port\n";
        }
        boost::asio::serial_port_base::baud_rate baud_option(baud);
        serialPort.set_option(baud_option); // set the baud rate after the port has been opened
    }

    void read(){

        read_start();
    }

    void ioservice_run(void){
        io_service_.run();

    }

    void write(char *msg) // pass the write data to the do_write function via the io service in the other thread
    {
        cout << "minicom.write: i got stuff " << msg << " and length=" << strlen(msg) << endl;
        if ( (isalpha(msg[0]) || isdigit(msg[0])) && (isalpha(msg[1]) || isdigit(msg[1])) ){
            cout << "check what yp" << endl;
            cmd[0]=msg[0];
            cmd[1]=msg[1];
            cout << "minicom.write: i got stuff " << cmd << " and length=" << strlen(cmd) << endl;
            io_service_.post(boost::bind(&minicom_client::do_write, this, cmd));
        }
    }

    void close() // call the do_close function via the io service in the other thread
        {
                io_service_.post(boost::bind(&minicom_client::do_close, this, boost::system::error_code()));
        }

        bool active() // return true if the socket is still active
        {
                return active_;
        }


private:

        static const int max_read_length = 512; // maximum amount of data to read in one operation

        void read_start(void)
        { // Start an asynchronous read and call read_complete when it completes or fails
                serialPort.async_read_some(boost::asio::buffer(read_msg_, max_read_length),
                        boost::bind(&minicom_client::read_complete,
                                this,
                                boost::asio::placeholders::error,
                                boost::asio::placeholders::bytes_transferred));
        }

        void read_complete(const boost::system::error_code& error, size_t bytes_transferred)
        { // the asynchronous read operation has now completed or failed and returned an error
                if (!error)
                { // read completed, so process the data
                    //cout.write(read_msg_, bytes_transferred); // echo to standard output
                     if (bytes_transferred == 12){
                         state_.micro_.set_parameters(std::string(&read_msg_[0],&read_msg_[11]));
                     }
                     read_start(); // start waiting for another asynchronous read again
                }
                else
                        do_close(error);
        }

        void do_write(char *msg)
        { // callback to handle write call from outside this class
                bool write_in_progress = !write_msgs_.empty(); // is there anything currently being written?
                // for(unsigned int i=0;i<strlen(msg);i++){
                //     write_msgs_.push_back(msg[i]); // store in write buffer
                // }
                cout << "minicom.do_write: i got stuff " << cmd << endl;
                write_msgs_.push_back(msg[0]);
                write_msgs_.push_back(msg[1]);
                if (!write_in_progress) // if nothing is currently being written, then start
                        write_start();
        }

        void write_start(void)
        { // Start an asynchronous write and call write_complete when it completes or fails
            cout << "gonna write" << write_msgs_.front() << endl;
                boost::asio::async_write(serialPort,
                        boost::asio::buffer(&write_msgs_.front(), 2),
                        boost::bind(&minicom_client::write_complete,
                                    this,
                                    boost::asio::placeholders::error));
        }

    void write_complete(const boost::system::error_code& error)
        { // the asynchronous read operation has now completed or failed and returned an error
                if (!error)
                { // write completed, so send next write data

                    write_msgs_.pop_front(); // remove the completed data
                    write_msgs_.pop_front(); // remove the completed data

                    if (!write_msgs_.empty()) // if there is anthing left to be written
                        write_start(); // then start sending the next item in the buffer
                }
                else
                        do_close(error);
        }

        void do_close(const boost::system::error_code& error)
        { // something has gone wrong, so close the socket & make this object inactive
                if (error == boost::asio::error::operation_aborted) // if this call is the result of a timer cancel()
                        return; // ignore it because the connection cancelled the timer
                if (error)
                        cerr << "Error: " << error.message() << endl; // show the error message
                else
                        cout << "Error: Connection did not succeed.\n";
                cout << "Press Enter to exit\n";
                serialPort.close();
                active_ = false;
        }


private:
    bool active_; // remains true while this object is still operating
    boost::asio::io_service& io_service_; // the main IO service that runs this connection
    boost::asio::serial_port serialPort; // the serial port this instance is connected to
    char read_msg_[max_read_length]; // data read from the socket
    char cmd[2];
    deque<char> write_msgs_; // buffered write data
    nodeState& state_;
};


void taskRead(minicom_client *c){
    c->read();
}


/*

  KEYBOARD INPUT

*/

void taskWrite(minicom_client *c){
    while (c->active()) // check the internal state of the connection to make sure it's still running
    {
        string input = "";
        char send[2];
        getline(cin,input);
        cout << "input: " << input[0] << input[1] << endl;
        send[0]=input[0];
        send[1]=input[1];
        c->write(send);
    }
    c->close(); // close the minicom client connection
}

int occupancy[8];
std::vector<udpClient*> neighborClients;


void taskEchoState(udpClient *client,nodeState *state,int mode)
{
    if (mode == 0) return;      // mode=0 -> centralized control = no need for P2P communication
    for(;;)
    {
        client->echo(state->micro_.getString());
        client->echo(state->getOccString());
        usleep(100000);
    }

}

int main(int argc, char* argv[])
{
    // on Unix POSIX based systems, turn off line buffering of input, so cin.get() returns after every keypress
    // On other systems, you'll need to look for an equivalent
#ifdef POSIX
    termios stored_settings;
    tcgetattr(0, &stored_settings);
    termios new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_lflag &= (~ISIG); // don't automatically handle control-C
    tcsetattr(0, TCSANOW, &new_settings);
#endif

    try
    {
        int myPort,neighbour1Port,neighbour2Port;
        std::string neighbour1Add,neighbour2Add;
        std::string neighborAddFirst, neighborAddSecond, token = ":";
        int mode;

        if (argc < 4 || argc > 6 || argc == 5)
        {
            cerr << "Usage: minicom <baud> <device> <port>\n" << endl;
            cerr << "Usage: minicom <baud> <device> <port> <first neighbor adress:port> <second neighbor address:port>\n";
            return 1;
        }
        // if there are 6 arguments then we're in distributed mode
        else if (argc == 6){
            mode = 1;           // mode=1 means distributed

            // get neighbour1's address and port
            neighbour1Add = std::string(argv[4]);
            neighbour1Port = atoi(neighbour1Add.substr(neighbour1Add.find(token)+1,5).c_str());
            neighbour1Add = neighbour1Add.substr(0,neighbour1Add.find(token));

            // get neighbour1's address and port
            neighbour2Add = std::string(argv[5]);
            neighbour2Port = atoi(neighbour2Add.substr(neighbour2Add.find(token)+1,5).c_str());
            neighbour2Add = neighbour2Add.substr(0,neighbour2Add.find(token));

            std::cout << "Neighbour 1\nPort:\t" << neighbour1Port << "\tAddress:\t" << neighbour1Add << std::endl;
            std::cout << "Neighbour 2\nPort:\t" << neighbour2Port << "\tAddress:\t" << neighbour2Add << std::endl;

        }
        // if there are 4 arguments then we're in centralized mode
        else if (argc == 4){
            mode = 0;
        }

        // get my port from argument 3
        myPort = atoi(argv[3]);
        cout << "My port is " << myPort << endl;

        boost::asio::io_service io_service;

        nodeState state(myPort,neighbour1Add,neighbour2Add);

        // define an instance of the main class of this program
        minicom_client c(io_service, boost::lexical_cast<unsigned int>(argv[1]), argv[2], state);

        // run the IO service as a separate thread, so the main thread can block on standard input
        //                thread t(&boost::asio::io_service::run, &io_service);


        thread tRead(taskRead,&c);    // thread to read from arduino
        thread tKeyboard(taskWrite,&c);  // thread to receive from keyboard and write to arduino



        udp_server server(io_service, myPort,state);

        std::thread tIO(boost::bind(&boost::asio::io_service::run, &io_service)); // thread for running the io service

        udpClient cNeighbour1(io_service,neighbour1Add,neighbour1Port);
        udpClient cNeighbour2(io_service,neighbour2Add,neighbour2Port);

        //mode=1

        std::thread tNeighbour1(boost::bind(taskEchoState,&cNeighbour1,&state,mode));
        std::thread tNeighbour2(boost::bind(taskEchoState,&cNeighbour2,&state,mode));


        while(true){ //slow print loop
            std::cout << "================= MY ARDUINO =================" << std::endl;

            cout << "\n\n=============== MY ARDUINO VALUES ===================" << endl;
            state.micro_.print();

            if (mode==1)
            {
                cout << "\n\n=============== NEIGHBOUR #1 VALUES ===================" << endl;
                state.micro1_.print();

                cout << "\n\n=============== NEIGHBOUR #2 VALUES ===================" << endl;
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
            }

            usleep(2000000);

        }

        tIO.join();

        tRead.join(); // wait for the IO service thread to close
        tKeyboard.join();

        tNeighbour1.join();
        tNeighbour2.join();

    }
    catch (exception& e)
    {
        cerr << "Exception: " << e.what() << "\n";
    }
#ifdef POSIX // restore default buffering of standard input
    tcsetattr(0, TCSANOW, &stored_settings);
#endif
    return 0;
}
