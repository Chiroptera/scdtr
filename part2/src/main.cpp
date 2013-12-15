 //  g++ -std=c++11 threadhello.cpp -lpthread -lboost_system-mt

#include <thread>
#include <iostream>
#include <iostream>
#include <string>
#include <sstream>
#include <boost/asio.hpp>
#include <boost/array.hpp>

#include <stdio.h>
#include "threadhello.h"

using namespace boost::system;
using namespace boost::asio;
using namespace std;

//GLOBALS
boost::asio::io_service io;
boost::asio::serial_port sp(io);
boost::asio::streambuf read_buf; //read buffer
string line;
Arduino arduino1;



int counter = 0;
//HANDLERS FOR ASYNC CALLBACKS
//declara@on of write_handler to @mer_handler
void write_handler(const error_code &ec,size_t nbytes);

void serialWrite(const error_code &ec) {
    //timer expired – launch new write opera@on
    ostringstream os;
    os << "Counter = " << ++counter;
    async_write(sp, buffer(os.str()), write_handler);
}

void write_handler(const error_code &ec, size_t nbytes){
    tim.expires_from_now(boost::posix_time::seconds(5));

}

void read_handler(const error_code &ec, size_t nbytes)
{
    //data is now avalilable at read_buf
    //cout << &read_buf;
    istream is(&read_buf);
    string line;
    getline(is, line);
    arduino1.set_parameters(line);
    arduino1.print();

    async_read_until(sp,read_buf,'\n',read_handler);
}

void Read_arduino()
{
    //program timer for write operations
    tim.expires_from_now(boost::posix_time::seconds(5));
    tim.async_wait(timer_handler);
    //program chain of read operations
    async_read_until(sp,read_buf,'\n',read_handler);
    io.run();
}

void Send_command()
{
while(1){
    //ostringstream os;
    string sender="99"; // this has to be replaced by a scanf function;
    //os << sender;
    async_write(sp, buffer(sender), write_handler);
    }
}

int main()
{
	char port[]="/dev/ttyACM0";	// shared by the two threads
  	boost::system::error_code ec;
    	sp.open(port,ec);		 //connect to port
    	sp.set_option(serial_port_base::baud_rate(115200),ec);

	thread t1(Read_arduino);
//	thread t2(Send_command);
//	cout << "Created child thread 1 #" <<t1.get_id() <<endl;
//	cout << "Created child thread 2 #" <<t2.get_id() <<endl;
	t1.join();	//wait until t1 finishes
//	t2.join();	//wait until t2 finishes

  // obj1.set_parameters(line);
	cout << "Finished." << endl;
	getchar();
}
