/* minicom.cpp
        A simple demonstration minicom client with Boost asio

        Parameters:
                baud rate
                serial port (eg /dev/ttyS0 or COM1)

        To end the application, send Ctrl-C on standard input
*/
#include <string>
#include <sstream>
#include "threadhello.h"
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
//#include <boost/enable_shared_from_this.hpp>

#include <deque>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <thread>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

#ifdef POSIX
#include <termios.h>
#endif


using namespace boost::asio;
using ip::udp;
using namespace std;
using boost::asio::ip::tcp;
using boost::asio::ip::udp;

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

        void write(const char *msg) // pass the write data to the do_write function via the io service in the other thread
        {

            if (strlen(msg)==2) io_service_.post(boost::bind(&minicom_client::do_write, this, msg));
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

        void do_write(const char *msg)
        { // callback to handle write call from outside this class
                bool write_in_progress = !write_msgs_.empty(); // is there anything currently being written?
                // for(unsigned int i=0;i<strlen(msg);i++){
                //     write_msgs_.push_back(msg[i]); // store in write buffer
                // }

                write_msgs_.push_back(msg[0]);
                write_msgs_.push_back(msg[1]);
                if (!write_in_progress) // if nothing is currently being written, then start
                        write_start();
        }

        void write_start(void)
        { // Start an asynchronous write and call write_complete when it completes or fails
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

        deque<char> write_msgs_; // buffered write data
        Arduino& arduino;
};


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
class tcp_server{
public:
tcp_server (boost::asio::io_service& io,minicom_client& micro)
    : microSerial(micro)
    {
    tcp::acceptor acceptor(io,tcp::endpoint(tcp::v4(),10000));
    boost::system::error_code err;
    for(;;)
    {
        tcp::socket socket(io);
        acceptor.accept(socket);
        std::cout << "conection made" << std::endl;
        //write(socket,buffer("Hello World\n"));
        for(;;){

            //        size_t len = socket.async_read_some(buffer(buf,2),err);
            socket.async_read_some(buffer(buf,2),boost::bind(&tcp_server::tcpRead,
                                                             this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));


            //socket.async_read_some(buffer(buf,2),tcpRead);

            if(err == error::eof)
                break;
            else if (err)
                std::cout << "Unknown Error";
            //            std::cout.write(buf.data(),len);
        }
    }
}

private:
    void tcpRead(const boost::system::error_code& error, size_t bytes_transferred){
        cout << "read " << bytes_transferred << endl;
         cout << "message " << buf[0] << buf[1] << endl;
        if (bytes_transferred==2){
            char cmd[2];
            cmd[0]=buf[0];
            cmd[1]=buf[1];
            microSerial.write(cmd);
        }
    }

private:
    boost::array<char,128> buf;
    minicom_client& microSerial;

};




void taskRead(minicom_client *c){
    c->read();
    c->ioservice_run();
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
                if (argc != 3)
                {
                        cerr << "Usage: minicom <baud> <device>\n";
                        return 1;
                }
                boost::asio::io_service io_service;
                Arduino arduino;
                // define an instance of the main class of this program
                minicom_client c(io_service, boost::lexical_cast<unsigned int>(argv[1]), argv[2], arduino);
                // run the IO service as a separate thread, so the main thread can block on standard input
                //                thread t(&boost::asio::io_service::run, &io_service);
                thread t(taskRead,&c);
                //boost::thread t(boost::bind(taskRead,&c));
                thread t2(taskWrite,&c);
                //udpClient x(2003,"127.0.0.1");
                tcp_server tcpstuff(io_service,c);

                // while(true){ //slow print loop
                //     //arduino.print();
                //     //cout << arduino.getString() << endl;
                //     x.write(arduino.getString());
                //     usleep(100000);
                // }

                t.join(); // wait for the IO service thread to close
                t2.join();
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
