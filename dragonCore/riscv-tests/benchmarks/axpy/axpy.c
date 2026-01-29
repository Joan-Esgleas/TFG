// See LICENSE for license details.

#include "syscalls.h"
#include <string.h>
#include <limits.h>

#define N_TIMES		10 /* Default number of runs */
#define DATA_SIZE 512
#define type_t int

type_t dx[DATA_SIZE];
type_t dy_opt[DATA_SIZE];


void axpy(type_t a, type_t *dx, type_t *dy, int n) {
   int i;
   for (i=0; i<n; i++) {
      dy[i] += a*dx[i];
   }
}

void axpy_unroll(type_t a, type_t *dx, type_t *dy, int n) {
   int i;
   for (i=0; i<n-6; i+=6) {
      type_t a0 = dx[i];
      type_t b0 = dy[i];
      type_t a1 = dx[i+1];
      type_t b1 = dy[i+1];
      type_t a2 = dx[i+2];
      type_t b2 = dy[i+2];
      type_t a3 = dx[i+3];
      type_t b3 = dy[i+3];
      type_t a4 = dx[i+4];
      type_t b4 = dy[i+4];
      type_t a5 = dx[i+5];
      type_t b5 = dy[i+5];
      type_t r0 = a*a0 + b0;
      type_t r1 = a*a1 + b1;
      type_t r2 = a*a2 + b2;
      type_t r3 = a*a3 + b3;
      type_t r4 = a*a4 + b4;
      type_t r5 = a*a5 + b5;
      dy[i] = r0;
      dy[i+1] = r1;
      dy[i+2] = r2;
      dy[i+3] = r3;
      dy[i+4] = r4;
      dy[i+5] = r5;
   }
   for (i; i<n; i++) {
      dy[i] += a*dx[i];
   }
}

void init_vector(type_t *pv, long n, type_t value)
{
   for (int i=0; i<n; i++){ 
     pv[i]= value;
   }
}


//--------------------------------------------------------------------------
// Main
//--------------------------------------------------------------------------

int main( int argc, char* argv[] ){

   printf("\n   *** AXPY BENCHMARK TEST ***\n\n");
   printf("Size of the vector:%d\n",DATA_SIZE);
   printf("Number of iterations:%d\n",N_TIMES);

   type_t a=(type_t)1;

   /* Allocate the source and result vectors */

   init_vector(dx, DATA_SIZE, (type_t)1);
   init_vector(dy_opt, DATA_SIZE, (type_t)2);
   
   setStats(1);
   for(int i = 0; i < N_TIMES; i++){
      axpy_unroll(a, dx, dy_opt, DATA_SIZE);
   }
   setStats(0);

   //for (int i = 0; i < DATA_SIZE; i++){
   //   printf("v[%d]= %d\n", i, (long long)dy_opt[i]);
   //}

   return 0;
}
