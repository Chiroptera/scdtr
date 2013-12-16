# ifndef MINICOMNCLASS_H
# define MINICOMNCLASS_H


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

#ifdef POSIX
#include <termios.h>
#endif

class minicom_client{
 public:
    minicom_client(boost::asio::io_service& io_service, unsigned int baud, const string& device, nodeState *state);
    void read();
    void ioservice_run(void);
    void write(char *msg);
    void close();
    bool active();
 private:

    static const int max_read_length = 512; // maximum amount of data to read in one operation

    void read_start(void);
    void read_complete(const boost::system::error_code& error, size_t bytes_transferred);
    void do_write(char *msg);
    void write_start(void);
    void write_complete(const boost::system::error_code& error);
    void do_close(const boost::system::error_code& error);

 private:

    bool active_; // remains true while this object is still operating
    boost::asio::io_service& io_service_; // the main IO service that runs this connection
    boost::asio::serial_port serialPort; // the serial port this instance is connected to
    char read_msg_[max_read_length]; // data read from the socket
    char cmd[3];
    deque<char> write_msgs_; // buffered write data
    nodeState *state_;
};

#endif
