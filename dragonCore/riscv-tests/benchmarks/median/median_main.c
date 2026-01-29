// See LICENSE for license details.

//**************************************************************************
// Median filter bencmark
//--------------------------------------------------------------------------
//
// This benchmark performs a 1D three element median filter. The
// input data (and reference data) should be generated using the
// median_gendata.pl perl script and dumped to a file named
// dataset1.h.

#include "syscalls.h"
#include "median.h"

#define NUMBER_OF_RUNS		10 /* Default number of runs */

//--------------------------------------------------------------------------
// Input/Reference Data

#include "dataset1.h"

//--------------------------------------------------------------------------
// Main

int main( int argc, char* argv[] )
{
  int results_data[DATA_SIZE];

#if PREALLOCATE
  // If needed we preallocate everything in the caches
  median( DATA_SIZE, input_data, results_data );
#endif

  // Do the filter
  setStats(1);
  for(unsigned int i = 0; i < NUMBER_OF_RUNS; i++) {
    median( DATA_SIZE, input_data, results_data );
  }
  setStats(0);

  // Check the results
  return verify( DATA_SIZE, results_data, verify_data );
}
