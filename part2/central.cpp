#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <thread>
#include <deque>
#include <string>
#include <ctype.h>
//#include "threadhello.h"


#include <sstream>
#include <iostream>
#include <boost/lexical_cast.hpp>


using namespace boost::asio;
using ip::udp;
using namespace std;
using boost::asio::ip::tcp;



// class udpServer{
// public:
//     udpServer(boost::asio::io_service& io, Arduino& arduino)
//         : io_service_(io),
//           micro(arduino),
//           socket_(io,udp::endpoint(udp::v4(),10000))
//     {

//     }

//     void respond(){

//         boost::array<char,1> recv;
//         udp::endpoint client;
//         boost::system::error_code err;
//         socket_.receive_from(buffer(recv),client,0,err);
//         if(err && err != error::message_size){
//             std::cout << "Error" << std::endl;
//         }
//         boost::system::error_code ignored;
//         socket_.send_to(buffer(micro.getString()),client,0,ignored);
//     }

// private:
//     boost::asio::io_service& io_service_;
//     udp::socket socket_;
//     Arduino& micro;
// };

// class tcp_server{
// public:
//     tcp_server (boost::asio::io_service& io, minicom_client *micro)
//         : io_(io),
//           microSerial(micro),
//                              active_(true)
//     {}

// void start(){
//     tcp::acceptor acceptor(io_,tcp::endpoint(tcp::v4(),10000));
//     boost::system::error_code err;
//     for(;;)
//     {
//         tcp::socket socket(io_);
//         acceptor.accept(socket);
//         std::cout << "conection made" << std::endl;
//         while(active_){
//             socket.async_read_some(buffer(buf),boost::bind(&tcp_server::tcpRead,
//                                                            this,
//                                                            boost::asio::placeholders::error,
//                                                            boost::asio::placeholders::bytes_transferred));
//         }
//     }
// }

// private:
//     void tcpRead(const boost::system::error_code& err, size_t bytes_transferred){

//         if(err == error::eof)
//             active_=false;
//         else if (err)
//             std::cout << "Unknown Error";

//         cout << "read " << bytes_transferred << endl;
//         cout << "message " << buf[0] << buf[1] << endl;
//         if (bytes_transferred==2){
//             char cmd[2];
//             cmd[0]=buf[0];
//             cmd[1]=buf[1];
//             microSerial->write(cmd);
//         }
//     }

// private:
//     bool active_;
//     minicom_client *microSerial;
//     boost::array<char,128> buf;
//     boost::asio::io_service& io_;
// };

// class udpClient{
// public:
//     udpClient(boost::asio::io_service& io)
//         : _io(io),
//           resolver(io)
//     {}

//     void queryServer(string addr, string port){
//         udp::resolver::query query(udp::v4(),addr,port);
//         udp::endpoint receiver = *resolver.resolve(query);

//         socket.open(udp::v4());
//         boost::array<char,1> send_buf = {{0}};

//         socket.send_to(buffer(send_buf),receiver);
//         boost::array<char,128> recv_buf;
//         udp::endpoint sender;
//         size_t len = socket.receive_from(buffer(recv_buf),sender);
//         std::cout.write(recv_buf.data(),len);
//     }

// private:
//     boost::asio::io_service& _io;
//     udp::socket _socket;
//     udp::resolver resolver;
// };



int main(){
    boost::asio::io_service io;
    //udpClient x(io);
    udp::resolver resolver(io);
    udp::socket socket(io);

    string addr="127.0.0.1";

    //    string port="10000";
    int ports[2];
    ports[0]=10000;
    ports[1]=10001;

    for(int i=0;i<2;i++){
        //        x.queryServer("127.0.0.1","10000");

        udp::resolver::query query(udp::v4(),addr,std::to_string(ports[i]));
        udp::endpoint receiver = *resolver.resolve(query);


        socket.open(udp::v4());
        boost::array<char,1> send_buf = {{0}};

        socket.send_to(buffer(send_buf),receiver);
        boost::array<char,128> recv_buf;
        udp::endpoint sender;
        size_t len = socket.receive_from(buffer(recv_buf),sender);
        std::cout.write(recv_buf.data(),len);

        socket.close();
    }
    return 0;
}
