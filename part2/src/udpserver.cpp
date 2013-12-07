#include<iostream>
#include <boost/asio.hpp>

using namespace boost::asio;
using ip::udp;
using namespace std;

void err(char *s)
{
   perror(s);
   exit(1);
}

int main (int argc,char *argv[])
{
char mensagem[20];

int server_sock=socket(AF_INET,SOCK_DGRAM,0);

struct sockaddr_in server_addr;
struct sockaddr_in client_addr;
server_addr.sin_family=AF_INET;
server_addr.sin_port=htons(2013);
server_addr.sin_addr.s_addr=inet_addr("127.0.0.1");
//memset(&client_addr, 0, sizeof(client_addr));
int ret=bind(server_sock,(struct sockaddr *)&server_addr,sizeof(struct sockaddr));
if(ret == -1) cout<<"BIND ERROR!\n"; else cout<<"Bind successful\n";

char buffer[20]={'\0'};
strcpy(mensagem,"Daniel!");
int count=0;


/************ com este codigo o servidor vai ler uma msg. */
for(;;){
	count=recvfrom(server_sock,&buffer,20,0,(struct sockaddr*)&client_addr,(socklen_t *)&client_addr);

	if(count==-1)cout<<"RECV ERROR\n"<<endl;
	else {
	    cout<<"Success. client says: "<<buffer<<endl;
	}
//sleep(1);
}

/*****************************************************************************/




/******* Com este codigo o cliente le uma string do teclado e depois escreve no servidor. Não esta a funcionar.
sleep(5);

for(;;)
{
strcpy(buffer,mensagem);
count=sendto(server_sock,&buffer,20,0,(struct sockaddr*)&server_addr,sizeof(server_addr));

if(count == -1) cout<<"Send ERROR\n"<<endl;
else cout<<"Send successful"<<endl;

cin >> mensagem;
sleep(1);
}

***********************************************************************************/

close(server_sock);
return 0;
}
