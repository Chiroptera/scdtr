#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <boost/array.hpp>
#include <vector>
#include <deque>

#include <thread>

#include <assert.h>
#include <string>
#include <ctype.h>

#include <iostream>
#include <sstream>
#include <iomanip>

#include <boost/lexical_cast.hpp>

#include "nodeState.h"
#include "threadhello.h"
#include "udpserver.h"
#include "udpClientClass.h"
#include "minicomnClass.h"

#ifdef POSIX
#include <termios.h>
#endif


minicom_client::minicom_client(boost::asio::io_service& io_service, unsigned int baud, const string& device, nodeState *state)
        : active_(true),
          io_service_(io_service),
          serialPort(io_service, device),
          state_(state)

    {
        if (!serialPort.is_open())
        {
std::cerr << "Failed to open serial port\n";
        }
        boost::asio::serial_port_base::baud_rate baud_option(baud);
        serialPort.set_option(baud_option); // set the baud rate after the port has been opened
    }

void minicom_client::read(){

        read_start();
    }

void minicom_client::ioservice_run(void){
        io_service_.run();

    }

void minicom_client::write(char *msg) // pass the write data to the do_write function via the io service in the other thread
    {
std::cout << "minicom : write: i got stuff " << msg << " and length=" << strlen(msg) << std::endl;
        if ( (isalpha(msg[0]) || isdigit(msg[0])) && (isalpha(msg[1]) || isdigit(msg[1])) ){
            cmd[0]=msg[0];
            cmd[1]=msg[1];
            cmd[2]=' ';
std::cout << "minicom : write: i got stuff " << cmd << " and length=" << strlen(cmd) << std::endl;
            io_service_.post(boost::bind(&minicom_client::do_write, this, cmd));
        }
    }

void minicom_client::close() // call the do_close function via the io service in the other thread
    {
        io_service_.post(boost::bind(&minicom_client::do_close, this, boost::system::error_code()));
    }

bool minicom_client::active() // return true if the socket is still active
    {
        return active_;
    }


void minicom_client::read_start(void)
    { // Start an asynchronous read and call read_complete when it completes or fails
        serialPort.async_read_some(boost::asio::buffer(read_msg_, max_read_length),
                                   boost::bind(&minicom_client::read_complete,
                                               this,
                                               boost::asio::placeholders::error,
                                               boost::asio::placeholders::bytes_transferred));
    }

void minicom_client::read_complete(const boost::system::error_code& error, size_t bytes_transferred)
    { // the asynchronous read operation has now completed or failed and returned an error
        if (!error)
        { // read completed, so process the data
            //cout.write(read_msg_, bytes_transferred); // echo to standard output
            //cout << "GOT STUFF " << read_msg_ << " BYTES: " << bytes_transferred << endl;
            if (bytes_transferred == 13){
                state_->micro_.set_parameters(std::string(&read_msg_[0],&read_msg_[11]));
                state_->setMyOccupancy();
                read_start(); // start waiting for another asynchronous read again
            }
            else
                do_close(error);
        }
    }

void minicom_client::do_write(char *msg)
    { // callback to handle write call from outside this class
        bool write_in_progress = !write_msgs_.empty(); // is there anything currently being written?
        // for(unsigned int i=0;i<strlen(msg);i++){
        //     write_msgs_.push_back(msg[i]); // store in write buffer
        // }
        std::cout << "minicom.do_write: i got stuff " << cmd << std::endl;
        std::cout << "pushing " << msg << std::endl;
        write_msgs_.push_back(msg[0]);
        write_msgs_.push_back(msg[1]);
        write_msgs_.push_back(msg[3]);
        if (!write_in_progress) // if nothing is currently being written, then start
            write_start();
    }

void minicom_client::write_start(void)
    { // Start an asynchronous write and call write_complete when it completes or fails
        cout << "gonna write" << write_msgs_.front() << endl;

        boost::asio::async_write(serialPort,
                                 boost::asio::buffer(&write_msgs_.front(), 3),
                                 boost::bind(&minicom_client::write_complete,
                                             this,
                                             boost::asio::placeholders::error));
    }

void minicom_client::write_complete(const boost::system::error_code& error)
    { // the asynchronous read operation has now completed or failed and returned an error
        if (!error)
        { // write completed, so send next write data

            write_msgs_.pop_front(); // remove the completed data
            write_msgs_.pop_front(); // remove the completed data
            write_msgs_.pop_front(); // remove the completed data

            if (!write_msgs_.empty()) // if there is anthing left to be written
                write_start(); // then start sending the next item in the buffer
        }
        else
            do_close(error);
    }

void minicom_client::do_close(const boost::system::error_code& error)
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
