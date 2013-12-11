#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <unistd.h>
#define M 30		//maximum number of lines for simplex
#define N 30		//maximum number of colums for simplex
#define V 8		//number of variables to optimize
#define Lmin 	300	// Lum for empty desk
#define Lmax 	600	// Lum for desk occupied


using namespace std;

double opt_sol[V]={0};


int main(){
  double coupling[V][V];
  double occupancy[V] = {1,0,1,1,0,0,0,1};
  double B[V] = {100,100,100,100,100,100,100,100};

  //fill the matrix of the PWM LEDs as an identity matrix and the crossed influences with an example
  for(int i=0; i<V; i++){
    for(int j=0; j<V; j++){
      if(i==j) coupling[i][j]=1000;	
      else     coupling[i][j]=100;		
    }			
  }
  double desiredLux[V]={500,200,500,500,200,200,200,500};
  int left,right;
  for (int i=0;;i++){
    left = (i%8 == 0) ? 7 : i%8 - 1;
    right = (i%8 == 7) ? 0 : i%8 + 1;

    opt_sol[i%8] = desiredLux[i%8] - opt_sol[left]*coupling[left][i%8] - opt_sol[right]*coupling[right][i%8];
    opt_sol[i%8] = opt_sol[i%8] / coupling[i%8][i%8];
    if (i%8 == 7){
      for (int j=0;j<8;j++){
	cout << opt_sol[j] << ",";
      }
      cout << endl;
    }
    //    sleep(0);
  }


}
