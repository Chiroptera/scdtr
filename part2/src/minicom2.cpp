/* minicom.cpp
        A simple demonstration minicom client with Boost asio

        Parameters:
                baud rate
                serial port (eg /dev/ttyS0 or COM1)

        To end the application, send Ctrl-C on standard input
*/

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>
#include <vector>
#include <thread>
#include <deque>
#include <string>
#include <ctype.h>
#include "threadhello.h"
#include <boost/array.hpp>

#include <sstream>
#include <iostream>
#include <boost/lexical_cast.hpp>



#ifdef POSIX
#include <termios.h>
#endif


using namespace boost::asio;
using ip::udp;
using namespace std;
using boost::asio::ip::tcp;
using boost::asio::ip::udp;

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
	std::cout << "degub#6" << "inside udpClient " <<  std::endl;
        _socket.send_to(buffer(message.c_str(),strlen(message.c_str())),receiver);

        boost::array<char,128> recv_buf;
        udp::endpoint sender;
        size_t len = _socket.receive_from(buffer(recv_buf),sender);
	std::cout << "degub#6" << "inside udpClient after recv " <<  std::endl;
	_socket.close();

	//	std::cout << std::string(recv_buf.data()).substr(0,len) << std::endl;
        return (std::string(recv_buf.data()).substr(0,len));
    }

    void sendData(std::string message){
        udp::resolver::query query(udp::v4(),_addr,std::to_string(_port));
        udp::endpoint receiver = *_resolver.resolve(query);

	_socket.open(udp::v4());
	
	_socket.send_to(buffer(message.c_str(),strlen(message.c_str())),receiver);

	_socket.close();
    }

    std::string getData(){
        udp::resolver::query query(udp::v4(),_addr,std::to_string(_port));
        udp::endpoint receiver = *_resolver.resolve(query);

	_socket.open(udp::v4());
	
	boost::array<char,128> recv_buf;
        udp::endpoint sender;
        size_t len = _socket.receive_from(buffer(recv_buf),sender);
	std::cout << "degub#6" << "inside udpClient after recv " <<  std::endl;

	_socket.close();


    }

private:
    boost::asio::io_service& _io;
    udp::socket _socket;
    udp::resolver _resolver;
    std::string _addr;
    int _port;
};


class minicom_client
{
public:
    minicom_client(boost::asio::io_service& io_service, unsigned int baud, const string& device, Arduino& arduinoIn)
        : active_(true),
          io_service_(io_service),
          serialPort(io_service, device),
          arduino(arduinoIn)
    {
        if (!serialPort.is_open())
        {
            cerr << "Failed to open serial port\n";
        }
        boost::asio::serial_port_base::baud_rate baud_option(baud);
        serialPort.set_option(baud_option); // set the baud rate after the port has been opened
        //read_start();
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

    void ard_print(void){arduino.print();}

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
                         arduino.set_parameters(std::string(&read_msg_[0],&read_msg_[11]));
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
    Arduino& arduino;
};

class udpServer{
public:
  udpServer(boost::asio::io_service& io, Arduino& arduino, minicom_client *micro,int port)
        : io_service_(io),
          _microArd(arduino),
          socket_(io,udp::endpoint(udp::v4(),port)),
					microSerial(micro)

    {

    }

    void respond(){

        boost::array<char,20> recv;
        udp::endpoint client;
        boost::system::error_code err;

        socket_.receive_from(buffer(recv),client,0,err);
	std::cout << "I got " << recv.data() << std::endl;

	std::cout <<"received from " << recv.data() << std::endl;
	char send[2];
        send[0]=recv[0];
        send[1]=recv[1];
	microSerial->write(send);

        if(err && err != error::message_size){
            std::cout << "Error" << std::endl;
        }
        boost::system::error_code ignored;
        socket_.send_to(buffer(_microArd.getString()),client,0,ignored);
    }

private:
    boost::asio::io_service& io_service_;
    udp::socket socket_;
    Arduino& _microArd;
		minicom_client *microSerial;

};



void taskRead(minicom_client *c){
    c->read();
    c->ioservice_run();
}



void taskUDP(udpServer *udp){
    for(;;){
        udp->respond();
        //minicom_client *microSerial;
    }
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
boost::array<Arduino,2> neighborMicros;
std::vector<udpClient*> neighborClients;

void taskNeighbor(int neighborNumber){
	
for(;;)
	{
		usleep(10000);	
		std::string response ="";
		    
		std::cout << "================= NEIGHBOR #" << neighborNumber << " =================" << std::endl;
	   	response = neighborClients[neighborNumber]->queryServer("");
		std::cout << "got:" << response << std::endl;
		neighborMicros[neighborNumber].set_parameters(response);

		neighborMicros[neighborNumber].print();
	}

}

void taskNeighborSend(int neighborNumber){

    for(;;){
        usleep(1000000);

std::cout << "degub#6" << "inside thread " << neighborNumber <<  std::endl;
        neighborClients[neighborNumber]->sendData(neighborMicros[neighborNumber].getString());

	std::cout << "================================" << std::endl;
    }
}

void taskNeighborRecv(int neighborNumber){
for(;;){
        std::string response ="";
            
	std::cout << "================= NEIGHBOR #" << neighborNumber << " =================" << std::endl;
   	response = neighborClients[neighborNumber]->getData();
	std::cout << "got:" << response << std::endl;
	neighborMicros[neighborNumber].set_parameters(response);

	std::cout << "\n\nGot info from neighbor " << neighborNumber << response << std::endl;
	neighborMicros[neighborNumber].print();
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
int port, port_first, port_second;
 std::string neighborAddFirst, neighborAddSecond, Add1, Add2, port1,port2,token = ":";

  try
    {
      if (argc < 4 || argc > 6 || argc == 5)
	{
	  cerr << "Usage: minicom <baud> <device> <port>\n" << endl;
	  cerr << "Usage: minicom <baud> <device> <port> <first neighbor port> <second neighbor port>\n";
	  return 1;
	}
      else if (argc == 4){
	port = atoi(argv[3]);
      }
      else if (argc == 6){
          neighborAddFirst = std::string(argv[4]);
          port1 = neighborAddFirst.substr(neighborAddFirst.find(token)+1,5);
          Add1 = neighborAddFirst.substr(0,neighborAddFirst.find(token));
          port_first = atoi(port1.c_str());

          neighborAddSecond = std::string(argv[5]);
          port2 = neighborAddSecond.substr(neighborAddSecond.find(token)+1,5);
          Add2 = neighborAddSecond.substr(0,neighborAddSecond.find(token));
          port_second = atoi(port2.c_str());

          std::cout << "port 1 " << port_first << "\t port2 " << port_second << std::endl;

          std::cout << "add 1" << Add1 << "\t add2 " << Add2 << std::endl;

      }


      boost::asio::io_service io_service;
      Arduino arduino;
      // define an instance of the main class of this program
      minicom_client c(io_service, boost::lexical_cast<unsigned int>(argv[1]), argv[2], arduino);
      // run the IO service as a separate thread, so the main thread can block on standard input
      //                thread t(&boost::asio::io_service::run, &io_service);


      thread t(taskRead,&c);    // thread to read from arduino
      thread t2(taskWrite,&c);  // thread to receive from keyboard and write to arduino

      udpServer x(io_service,arduino,&c,port); // udpServer to handle udp requests
      thread tUDP(taskUDP,&x);                 // thread to run the udp server

      // if running on distributed mode
      if (argc == 6){
	std::cout << "tenho 6 pars e vou criar clients e threads" << std::endl;
          neighborClients.push_back(new udpClient(io_service,Add1,port_first));
          neighborClients.push_back(new udpClient(io_service,Add2,port_second));
     std::cout << "degub#2" << std::endl;
   
     }



std::cout << "degub#3" << std::endl;
	  thread neighborFirst(taskNeighbor,0);
	  thread neighborSecond(taskNeighbor,1);
          //thread neighborFirstSend(taskNeighborSend,0);
          //thread neighborFirstRecv(taskNeighborRecv,0);
          //thread neighborSecondSend(taskNeighborSend,1);
	  //thread neighborSecondRecv(taskNeighborRecv,1);

std::cout << "degub#5" << std::endl;
      while(true){ //slow print loop
	std::cout << "================= MY ARDUINO =================" << std::endl;
	arduino.print();
	//     //cout << arduino.getString() << endl;
	//     x.write(arduino.getString());




	usleep(1000000);
      }

      t.join(); // wait for the IO service thread to close
      t2.join();
      tUDP.join();
      neighborFirst.join(); neighborSecond.join();
      //neighborFirstSend.join();neighborFirstRecv.join();
      //neighborSecondSend.join();neighborSecondRecv.join();
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
