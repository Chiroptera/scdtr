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

#define M 30		//maximum number of lines for simplex
#define N 30		//maximum number of colums for simplex
#define V 8		//number of variables to optimize
#define Lmin 	1	// Lum for empty desk
#define Lmax 	10.9	// Lum for desk occupied

// convertion to lnux constants
const float ep = 0.1;
const int Rlux = 100000;

// conditions of central
bool testMode = false;
const int NumberOfClients = 8;

// containers for arduinos and communication clients
boost::array<Arduino,8> micros;

//simplex stuff
static const double epsilon   = 1.0e-8;
int equal(double a, double b) { return fabs(a-b) < epsilon; }

typedef struct {
int m, n; // m=rows, n=columns, mat[m x n]
double mat[M][N];
} Tableau;

double opt_sol[NumberOfClients]; //values to send to the arduinos



using namespace boost::asio;
using namespace std;



/*

SIMPLEX

*/



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


void nl(int k){
	int j;
	for(j=0;j<k;j++) putchar('-');
	putchar('\n');
}

void get_optimal_vector_clean(Tableau *tab) {
    int j, k;
    k=tab->n-V;
    for(j=tab->n-V; j<tab->n; j++){ // for each line of the slack variables.
        opt_sol[j-k]=tab->mat[0][j];
        tab->mat[0][j]=0;
    }
}


void print_tableau(Tableau *tab) {
  static int counter=0;
  int i, j;
  nl(70);

  printf("%-6s%5s", "col:", "b[i]");
  for(j=1;j<tab->n; j++) { printf("    x%d,", j); } printf("\n");

  for(i=0;i<tab->m; i++) {
    if (i==0) printf("max:"); else
    printf("b%d: ", i);
    for(j=0;j<tab->n; j++) {
      if (equal((int)tab->mat[i][j], tab->mat[i][j]))
        printf(" %6d", (int)tab->mat[i][j]);
      else
        printf(" %6.2lf", tab->mat[i][j]);
      }
    printf("\n");
  }
  nl(70);
}



int simplexDummy(float E[][NumberOfClients],float O[NumberOfClients],int occupancy[NumberOfClients])
{

Tableau tab;

int i, j;

int Ldes[NumberOfClients];		// Desired luminace in each desk
float b[NumberOfClients];		// luminance to provide in each desk
//float E[NumberOfClients][NumberOfClients];		// crossed influences
float E_t[NumberOfClients][NumberOfClients];	// the transposed of the crossed influences

int cost[NumberOfClients]={1,1,1,1,1,1,1,1};			   // cost -> in the simplex they will be constraints: >= 1 (all LEDs penalized equaly)
//int occupancy[NumberOfClients]={1, 0, 1, 1, 0, 0, 0, 1};	   // states of the desks -> free or occupied (example)
//int O[NumberOfClients]={100, 100, 100, 100, 100, 100, 100, 100};  //environment luminance


//fill the matrix of the PWM LEDs as an identity matrix and the crossed influences with an example
// for(i=0; i<NumberOfClients; i++){
// 	for(j=0; j<NumberOfClients; j++){
// 		if(i==j) E[i][j]=1000;
// 		else     E[i][j]=100;
// 	}
// }

// calculate the luminance we need to provide in each desk
for(i=0; i<NumberOfClients; i++){
	if(occupancy[i]==0) Ldes[i]=Lmin; 	// desk free
	else Ldes[i]=Lmax;			// desk occupied
	b[i]=Ldes[i]-O[i];			// luminace constraint result L-O
}

// create the transpose of E
for(i=0; i<NumberOfClients; i++){
	for(j=0; j<NumberOfClients; j++){
		E_t[i][j]=E[j][i];}
}



tab.m=NumberOfClients+1;	// number of conditons + 1 (cost_function)
tab.n=2*NumberOfClients+1;	// number of variables + 2 (b e -v)

/********************************************************************/
//Construction of the tableu based on the data of the problem;

for(i=0; i<tab.m; i++){
	for(j=0; j<9; j++){
		if(i==0 && j==0) tab.mat[0][0]=0;
		if(i==0 && j!=0) tab.mat[0][j]=b[j-1];
		if(i!=0 && j==0) tab.mat[i][0]=cost[i-1];
		if(i!=0 && i<=NumberOfClients && j!=0) tab.mat[i][j]=E_t[i-1][j-1];
		}
}


//add the -v variable to ensure the limite pwm<=1. Without this the Optimal solution can have leds with pwm higher than 1 !!!
for(i=0; i<tab.m; i++){
	for(j=9; j<tab.n; j++){
		if(i==j-8) tab.mat[i][j]=-1;
		else tab.mat[i][j]=0;
		}
	}
for(i=9; i<17; i++) tab.mat[0][i]=-1;

// change signs of the first row
for(i=0; i<tab.n; i++) tab.mat[0][i]= -tab.mat[0][i];

/*******************************************************************/

  print_tableau(&tab);
  simplex(&tab);
  printf("Solution: ");
  for(j=0; j<NumberOfClients; j++) printf("X%d=%3.6lf ",j+1,opt_sol[j]);
  printf("\n");


/***** clean slack variables and clean solution **/


  return 0;
}


/*

END OF SIMPLEX

*/






void taskRead(minicom_client *c){
    c->read();
    c->ioservice_run();
}

/*

  KEYBOARD INPUT

*/

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

void taskEchoState(udpClient *client,nodeState *state,int mode)
{
    if (mode == 0) return;      // mode=0 -> centralized control = no need for P2P communication
    for(;;)
    {
        client->echo(state->micro_.getString());
        client->echo(state->getOccString());
        usleep(100000);
    }

}

void taskServerUDP(udp_server *server)
{
    server->start_receive();
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

        int myPort,neighbour1Port,neighbour2Port;
        std::string neighbour1Add,neighbour2Add;
        std::string neighborAddFirst, neighborAddSecond, token = ":";
        int mode;

        std::cout << "ARGS: " << argc << std::endl;
        if (argc != 4 && argc != 6)
        {
            cerr << "Usage: minicom <baud> <device> <port>\n" << endl;
            cerr << "Usage: minicom <baud> <device> <port> <first neighbor adress:port> <second neighbor address:port>\n";
            return 1;
        }

        // if there are 6 arguments then we're in distributed mode
        else if (argc == 6){
            mode = 1;           // mode=1 means distributed

            // get neighbour1's address and port
            neighbour1Add = std::string(argv[4]);
            neighbour1Port = atoi(neighbour1Add.substr(neighbour1Add.find(token)+1,5).c_str());
            neighbour1Add = neighbour1Add.substr(0,neighbour1Add.find(token));

            // get neighbour1's address and port
            neighbour2Add = std::string(argv[5]);
            neighbour2Port = atoi(neighbour2Add.substr(neighbour2Add.find(token)+1,5).c_str());
            neighbour2Add = neighbour2Add.substr(0,neighbour2Add.find(token));

            std::cout << "Neighbour 1\nPort:\t" << neighbour1Port << "\tAddress:\t" << neighbour1Add << std::endl;
            std::cout << "Neighbour 2\nPort:\t" << neighbour2Port << "\tAddress:\t" << neighbour2Add << std::endl;


        }
        // if there are 4 arguments then we're in centralized mode
        else if (argc == 4){
            mode = 0;
        }

        // get my port from argument 3
        myPort = atoi(argv[3]);
        cout << "My port is " << myPort << endl;

        boost::asio::io_service io_service;

        nodeState state(myPort,neighbour1Add,neighbour2Add);

        // define an instance of the main class of this program
        minicom_client c(io_service, boost::lexical_cast<unsigned int>(argv[1]), argv[2], &state);

        // run the IO service as a separate thread, so the main thread can block on standard input
        //                thread t(&boost::asio::io_service::run, &io_service);


        thread tRead(taskRead,&c);    // thread to read from arduino
        thread tKeyboard(taskWrite,&c);  // thread to receive from keyboard and write to arduino

        udp_server server(io_service, myPort,state,&c);

        // std::thread tIO(boost::bind(&boost::asio::io_service::run, &io_service)); // thread for running the io service

        std::thread UDPServer(boost::bind(taskServerUDP,&server));

        udpClient cNeighbour1(io_service,neighbour1Add,neighbour1Port);
        udpClient cNeighbour2(io_service,neighbour2Add,neighbour2Port);

        //mode=1

        std::thread tNeighbour1(boost::bind(taskEchoState,&cNeighbour1,&state,mode));
        std::thread tNeighbour2(boost::bind(taskEchoState,&cNeighbour2,&state,mode));


        while(true){ //slow print loop
            std::cout << "================= MY ARDUINO =================" << std::endl;

            cout << "\n\n=============== MY ARDUINO VALUES ===================" << endl;
            state.micro_.print();

            // if (state.toWrite_==true){
            //     state.toWrite_=false;
            //     c.write(state.send);
            // }


            if (mode==1)
            {

                if (state.ready_){
                    simplexDummy(state.coupling_,state.background_,state.occupancy_);
                    int valueToSend = (int) opt_sol[state.myOccPos] * 255;
                    std::stringstream stream;
                    stream << std::setfill('0') << std::setw(2) << std::hex << valueToSend;
                    std::string msg = stream.str();
                    char send[2];
                    send[0]=msg[0]; send[1]=msg[1];
                    c.write(send);
                }

                cout << "\n\n=============== NEIGHBOUR #1 VALUES ===================" << endl;
                state.micro1_.print();

                cout << "\n\n=============== NEIGHBOUR #2 VALUES ===================" << endl;
                state.micro2_.print();


                cout << "\nCOUPLING" << endl;
                for (int i=0;i<8;i++)
                {
                    for (int j=0;j<8;j++)
                    {
                        cout << state.getCoupling(i,j) << ",";
                    }
                    cout << endl;
                }

                cout << "\n\nBACKGROUND" << endl;
                for (int i=0;i<8;i++){cout << state.getBackground(i) << ",";}
                cout << endl;
            }

            cout << "\n\nOCCUPANCY" << endl;
            cout << " my occ is " << state.micro_.getPresence() << endl;
            for (int i=0;i<8;i++){cout << state.getOccupancy(i) << ",";}
            cout << endl;


            usleep(1000000);

        }



        tRead.join(); // wait for the IO service thread to close
        tKeyboard.join();

        UDPServer.join();
        // tIO.join();

        tNeighbour1.join();
        tNeighbour2.join();

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
