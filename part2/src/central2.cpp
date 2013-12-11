#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <string>
#include "threadhello.h"
#include <vector>
#include <math.h>

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#define M 30		//maximum number of lines for simplex
#define N 30		//maximum number of colums for simplex
#define V 8		//number of variables to optimize
#define Lmin 	300	// Lum for empty desk
#define Lmax 	600	// Lum for desk occupied

using namespace boost::asio;
using namespace std;
using ip::udp;



bool testMode = false;
const int NumberOfClients = 8;
boost::array<Arduino,8> micros;


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

        _socket.send_to(buffer(message.c_str(),strlen(message.c_str())),receiver);
        boost::array<char,128> recv_buf;
        udp::endpoint sender;
        size_t len = _socket.receive_from(buffer(recv_buf),sender);
	_socket.close();

	//	std::cout << std::string(recv_buf.data()).substr(0,len) << std::endl;
        return (std::string(recv_buf.data()).substr(0,len));
    }

private:
  boost::asio::io_service& _io;
  udp::socket _socket;
  udp::resolver _resolver;
  std::string _addr;
  int _port;
};


std::vector<udpClient*> clients;

static const double epsilon   = 1.0e-8;
int equal(double a, double b) { return fabs(a-b) < epsilon; }

typedef struct {
  int m, n; // m=rows, n=columns, mat[m x n]
  double mat[M][N];
} Tableau;

double opt_sol[V];


void pivot_on(Tableau *tab, int row, int col) {
  int i, j;
  double pivot;

  pivot = tab->mat[row][col];
  assert(pivot>0);
  for(j=0;j<tab->n;j++)
    tab->mat[row][j] /= pivot;
  assert( equal(tab->mat[row][col], 1. ));

  for(i=0; i<tab->m; i++) { // foreach remaining row i do
    double multiplier = tab->mat[i][col];
    if(i==row) continue;
    for(j=0; j<tab->n; j++) { // r[i] = r[i] - z * r[row];
      tab->mat[i][j] -= multiplier * tab->mat[row][j];
    }
  }
}


// Find pivot_col = most negative column in mat[0][1..n]
int find_pivot_column(Tableau *tab) {
  int j, pivot_col = 1;
  double lowest = tab->mat[0][pivot_col];
  for(j=1; j<tab->n; j++) {
    if (tab->mat[0][j] < lowest) {
      lowest = tab->mat[0][j];
      pivot_col = j;
    }
  }
  if( lowest >= 0 ) {
    return -1; // All positive columns in row[0], this is optimal.
  }
  return pivot_col;
}


// Find the pivot_row, with smallest positive ratio = col[0] / col[pivot]
int find_pivot_row(Tableau *tab, int pivot_col) {
  int i, pivot_row = 0;
  double min_ratio = -1;
  for(i=1;i<tab->m;i++){
    double ratio = tab->mat[i][0] / tab->mat[i][pivot_col];
    if ( (ratio > 0  && ratio < min_ratio ) || min_ratio < 0 ) {
      min_ratio = ratio;
      pivot_row = i;
    }
  }
  if (min_ratio == -1)
    return -1; // Unbounded.

  return pivot_row;
}


void add_slack_variables(Tableau *tab) {
  int i, j;
  for(i=1; i<tab->m; i++) {
    for(j=1; j<tab->m; j++)
      tab->mat[i][j + tab->n -1] = (i==j);
  }
  tab->n += tab->m -1;
}


void check_b_positive(Tableau *tab) {
  int i;
  for(i=1; i<tab->m; i++)
    assert(tab->mat[i][0] >= 0);
}


// Given a column of identity matrix, find the row containing 1.
// return -1, if the column as not from an identity matrix.
int find_basis_variable(Tableau *tab, int col) {
  int i, xi=-1;
  for(i=1; i < tab->m; i++) {
    if (equal( tab->mat[i][col],1) ) {
      if (xi == -1)
        xi=i;   // found first '1', save this row number.
      else
        return -1; // found second '1', not an identity matrix.

    } else if (!equal( tab->mat[i][col],0) ) {
      return -1; // not an identity matrix column.
    }
  }
  return xi;
}



void get_optimal_vector(Tableau *tab) {
  int j, k;
  k=tab->n-V;
  for(j=tab->n-V; j<tab->n; j++){ // for each line of the slack variables.
      opt_sol[j-k]=tab->mat[0][j]; }
}


void simplex(Tableau *tab) {
  int loop=0;
  add_slack_variables(tab);
  check_b_positive(tab);
  while( ++loop ) {
    int pivot_col, pivot_row;

    pivot_col = find_pivot_column(tab);
    if( pivot_col < 0 ) {
      get_optimal_vector(tab);
      break;
    }

    pivot_row = find_pivot_row(tab, pivot_col);
    if (pivot_row < 0) {
      printf("unbounded (no pivot_row).\n");
      break;
    }

    pivot_on(tab, pivot_row, pivot_col);
    get_optimal_vector(tab);

    if(loop > 50) {
      printf("Too many iterations > %d.\n", loop);
      break;
    }
  }
}


int simplexDummy(double coupling[][NumberOfClients],double background[NumberOfClients],bool occupancy2[NumberOfClients],double commands[NumberOfClients])
{
Tableau tab;

tab.m=V+1;	// number of conditons + 1 (cost_function)	
tab.n=V+1;	// number of variables + 1 (b)

int i, j;
	
int Ldes[V];		// Desired luminace in each desk
int b[V];		// luminance to provide in each desk
float E[V][V];		// crossed influences
float E_t[V][V];	// the transposed of the crossed influences

int cost[V]={1,1,1,1,1,1,1,1};			   // cost -> in the simplex they will be constraints: >= 1 (all LEDs penalized equaly)
int occupancy[V]={1, 0, 1, 1, 0, 0, 0, 1};	   // states of the desks -> free or occupied (example)
int O[V]={100, 100, 100, 100, 100, 100, 100, 100};  //environment luminance


//fill the matrix of the PWM LEDs as an identity matrix and the crossed influences with an example
for(i=0; i<V; i++){
	for(j=0; j<V; j++){
		if(i==j) E[i][j]=1000;	
		else     E[i][j]=100;		
	}			
}

// calculate the luminance we need to provide in each desk
for(i=0; i<V; i++){
	if(occupancy[i]==0) Ldes[i]=Lmin; 	// desk free 
	else Ldes[i]=Lmax;			// desk occupied
	b[i]=Ldes[i]-O[i];			// luminace constraint result L-O
}

// create the transpose of E
for(i=0; i<V; i++){
	for(j=0; j<V; j++){
		E_t[i][j]=E[j][i];}
}


//Construction of the tableu based on the data of the problem
for(i=0; i<tab.m; i++){
	for(j=0; j<tab.n; j++){
		if(i==0 && j==0) tab.mat[0][0]=0;
		if(i==0 && j!=0) tab.mat[0][j]=b[j-1];
		if(i!=0 && j==0) tab.mat[i][0]=cost[i-1];
		if(i!=0 && i<=V && j!=0) tab.mat[i][j]=E_t[i-1][j-1];
		}
}		

// change signs of the first row
for(i=0; i<tab.n; i++) tab.mat[0][i]= -tab.mat[0][i];

  simplex(&tab);
  printf("Solution: ");
  for(j=0; j<V; j++) printf("X%d=%3.6lf ",j+1,opt_sol[j]);
  printf("\n");
  return 0;
} 


/******************************************


COMMUNICATION


 ******************************************/



void updateStates(bool occupancy[NumberOfClients]){
  int i=0;
  while(i<NumberOfClients){
    std::string response = "";

    std::cout << "querying in client " << i << endl;
    response = clients[i]->queryServer("");
    micros[i].set_parameters(response.substr(0,11));
    micros[i].print();
    occupancy[i]=micros[i].getPresence();
    i++;
  }
  return;
}

void getBackgroundAndCoupling(double coupling[][NumberOfClients],double background[NumberOfClients],bool occupancy[NumberOfClients]){

  // initialize all workstations with LED at 0% PWM
  for (int i=0;i<NumberOfClients;i++){
      clients[i]->queryServer("00");
  }

  // update for background data
  updateStates(occupancy);

  double aux;

  // get background data
  std::cout << "\n\n\n\n doing background \n\n\n";
  for (int i=0;i<NumberOfClients;i++){
    aux = 198 - micros[i].getLDR();
    aux = aux / 33;
    background[i] = pow(10.00,aux);
  }


  // coupling matrix
  std::cout << "\n\n\n\n doing coupling \n\n\n";
  for (int i=0;i<NumberOfClients;i++){

    // change LED i to 01
    clients[i]->queryServer("01");

    // update all info
    updateStates(occupancy);

    // update coupling matrix
    for (int j=0;j<NumberOfClients; j++){
      std::cout << "effect of micro " << i << " in micro " << j << " is " << micros[j].getLDR() << std::endl;
      aux = 198 - micros[j].getLDR();
      aux = aux / 33;
      coupling[i][j] = pow(10.00,aux);
    }

    // restore LED i to 00
    clients[i]->queryServer("00");
  }


}



int main(int argc, char **argv)
{

  if (argc == 2){
    testMode = true;
  }

   io_service io;
   udp::resolver resolver(io);

   double coupling[NumberOfClients][NumberOfClients]={{0}};
   double background[NumberOfClients]={0};
   bool occupancy2[NumberOfClients]={0};
   double commands[NumberOfClients];

   std::string addrs[NumberOfClients+1];
   addrs[0] = "192.168.27.202";
   addrs[1] = "192.168.27.204";
   addrs[2] = "192.168.27.203";
   addrs[3] = "192.168.27.206";
   addrs[4] = "192.168.27.205";
   addrs[5] = "192.168.27.207";
   addrs[6] = "192.168.27.208";
   addrs[7] = "192.168.27.209";
   addrs[8] = "192.168.27.201"; //professor computer

   int ports[NumberOfClients];
   ports[0]=17231;
   ports[1]=17232;
   ports[2]=17233;
   ports[3]=17234;
   ports[4]=17235;
   ports[5]=17236;
   ports[6]=17237;
   ports[7]=17238;




   for (int i=0;i<NumberOfClients;i++){
     if (testMode) clients.push_back(new udpClient(io,"127.0.0.1",ports[i]));
     else  clients.push_back(new udpClient(io,addrs[i],ports[i]));
   }


   //   udpClient central(io);
   std::cout << "starting update" << endl;
   updateStates(occupancy2);
   std::cout << "update finished" << endl;

   getBackgroundAndCoupling(coupling,background,occupancy2);

   // print background matrix
   std::cout << "\n\nBACKGROUND MATRIX\n\n";
   for (int i=0; i<NumberOfClients;i++){
     std::cout << background[i] << ",";
   }
   std::cout << std::endl;

   // print coupling matrix
   std::cout << "\n\nCOUPLING MATRIX\n\n";
   for (int i=0; i<NumberOfClients;i++){
     std::cout << "\t";
     for (int j=0; j<NumberOfClients;j++){
       std::cout << coupling[i][j] << ",";
     }
     std::cout << std::endl;
   }

   

   simplexDummy(coupling,background,occupancy2,commands);
   
}
