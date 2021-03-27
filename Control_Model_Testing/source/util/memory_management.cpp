#include "memory_management.h"


/*
 Returns a storage array for the state values which are solved.
 */
double* memory_allocation_for_x_vector_storage_arrays(double sim_time, double dt){
    double *x_store;

    if(STATE_SPACE_MATRIX_SIZE == 2){
        x_store = (double *) malloc(2 * (sim_time*1/dt + 1)*sizeof(double));
    }
    if(STATE_SPACE_MATRIX_SIZE == 4){
        x_store = (double *) malloc(4 * (sim_time*1/dt + 1)*sizeof(double));
    }

    return x_store;
}


/*
 Returns a storage array for the u values.
 */
double* memory_allocation_for_u_vector_storage_arrays(double sim_time, double dt){
    double *x_store;

    /* Allocate memory for 1D array */
    x_store = (double *) malloc(sim_time*1/dt*sizeof(double));

    return x_store;
}


/*
 Frees the memory of used by the storage arrays.
 */
void destroy_arrays(double* array){
    free(array);
}
