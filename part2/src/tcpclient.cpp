#include <iostream>
#include <sstream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <string>
#include <unistd.h>
using namespace boost::asio;
using ip::tcp;

class tcp_client{
public:
  tcp_client (boost::asio::io_service& io, int port, std::string addr)
        : _io(io),
					_port(port),
					_addr(addr),
          _resolver(io),
          _socket(io)

    {}

void send(std::string message){
	tcp::resolver::query query(_addr,std::to_string(_port));
	tcp::resolver::iterator endpoint=_resolver.resolve(query);
	boost::system::error_code err;

  _socket.connect(*endpoint,err);
	int l=write(_socket,buffer(message.c_str(),strlen(message.c_str())));
  std::cout << "bytes sent" << l << std::endl;

}


private:
  boost::asio::io_service& _io;
  tcp::socket _socket;
  tcp::resolver _resolver;
  std::string _addr;
  int _port;
};

int main(){
	io_service io;
	boost::system::error_code err;
	tcp_client client(io,17000,"127.0.0.1");
	for(;;){
		client.send("22");
usleep(100000);
	}

}


