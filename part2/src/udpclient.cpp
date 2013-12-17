#include<iostream>
#include <string>
#include <boost/asio.hpp>

using namespace boost::asio;
using ip::udp;
using namespace std;

int main(int argc,char *argv[]){

int client_sock=socket(AF_INET,SOCK_DGRAM,0);

char mensagem[20];
struct sockaddr_in client_addr;
client_addr.sin_family=AF_INET;
client_addr.sin_port=htons(2013);
client_addr.sin_addr.s_addr=inet_addr("127.0.0.1");

socklen_t addr_len;
addr_len = sizeof(struct sockaddr);

char buffer[20]={'\0'};

strcpy(mensagem,"Daniel!");
int count=0;


/********* Com este codigo o cliente le uma string do teclado e depois escreve no servidor. */

for(;;)
{
strcpy(buffer,mensagem);
count=sendto(client_sock,&buffer,20,0,(struct sockaddr*)&client_addr,sizeof(client_addr));

if(count==-1) cout<<"Send ERROR\n";
else cout<<"Send successful"<<endl;

cin >> mensagem;
//sleep(1);
}

/***************************************************************************************/





/*********** Com esta configuração o cliente é que recebe os dados do servidor. Não esta a funcionar
cout<<"Client started"<<endl;

for(;;)
{
count=recvfrom(client_sock,&buffer,20,0,(struct sockaddr*)&client_addr,&addr_len);

if(count==-1)cout<<"RECV ERROR\n";
else{
    cout<<"Success. Server says:"<<buffer<<endl;
    }
sleep(1);
}

********************************************************************************/


close(client_sock);
return 0;
}
