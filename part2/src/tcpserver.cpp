#include <boost/asio.hpp>

using namespace boost::asio;
using ip::tcp;

int main()
{
	io_service io;

	tcp::acceptor acceptor(io,tcp::endpoint(tcp::v4(),10000));
        boost::system::error_code err;
	for(;;)
	{
		tcp::socket socket(io);
		acceptor.accept(socket);
                std::cout << "conection made" << std::endl;
		write(socket,buffer("Hello World\n"));
                for(;;){
                boost::array<char,128> buf;
		size_t len = socket.read_some(buffer(buf),err);

		if(err == error::eof)
                    break;
		else if (err)
                    std::cout << "Unknown Error";
		std::cout.write(buf.data(),len);}
	}
}

class TCPserver{
public:

private:
    boost::asio::io_service io;
    boost::system::error_code err;
    tcp::socket socket;

};
