#include <sys/types.h>
#include <sys/times.h>

#include "syscalls.h"

#include "dataset1.h"

int i;
int histogram[16] = {};

int cores_count = 1;
int core_data_lenght;

int main(int argc, char** argv) {

    printf("\n   *** HISTOGRAM BENCHMARK TEST ***\n\n");
    printf("N = %d \n", DATA_SIZE);

    // get the number of hardware cores we have available in the system
    cores_count = 1; //get_core_count();
    core_data_lenght = DATA_SIZE/cores_count;

    // Printing some info
    printf("Executing with : %u", cores_count);
    printf(" Cores\n");

    // wakeup all the cores
    //_cores_fork_(cores_count);

    // get the local core hardware id
    //int coreID = get_core_id();
    int coreID = 0;

    // Calculate the cores begin and end of the data chunks
    int begining = coreID * core_data_lenght;
    int ending = (coreID + 1) * core_data_lenght;


    setStats(1);

//-------------------------
	for(int x=begining; x<ending; x++){
		histogram[data[x]] += 1;
	}
//-------------------------

    setStats(0);  

    // synchronize the cores execution
    //barrier(cores_count);

    // put the core except the 0 to idle mode
    //_core_idle_(coreID);

    printf("Output : ");

    for (int x=0; x<16; x++)
	    printf("%u:%u,", x, histogram[x]);

    printf("\n");

    printf("Successful\n\n");

    return 0;
}
