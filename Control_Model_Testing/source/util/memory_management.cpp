#include "memory_management.h"


/*
Returns a storage array for the state values which are solved during the
simulation
*/
double** memory_allocation_for_storage_arrays(int sim_time, double dt){
    int i;
    double **x_store;

    /* Allocate memory for the 2D arrays */
    x_store = (double **) malloc(sim_time*1/dt*sizeof(double *));
    
    /* If memory could not be allocated */
    if(x_store == NULL){
        printf("Error: out of memory.\n");
    }

    /* Allocate memory for each sub array */
    for(i=0; i<sim_time*1/dt; i++){
        x_store[i] = (double *) malloc(STATE_SPACE_MATRIX_SIZE*sizeof(double));
        /* If memory could not be allocated */
        if(x_store[i] == NULL){
            printf("Error: out of memory.\n");
        }
    }
    return x_store;
}


/*
Frees the memory of used by the storage arrays.
*/
void destroy_arrays(double** array){
    free(*array);
    free(array);
}
