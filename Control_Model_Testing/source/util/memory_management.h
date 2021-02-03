#ifndef MEMORY_MANAGEMENT_H
#define MEMORY_MANAGEMENT_H

#include <stdio.h>
#include <stdlib.h>


double** memory_allocation_for_storage_arrays(double sim_time, double dt);

double* memory_allocation_for_1_d_arrays(double sim_time, double dt);

void destroy_arrays(double** array);

#endif
