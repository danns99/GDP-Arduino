#ifndef MEMORY_MANAGEMENT_H
#define MEMORY_MANAGEMENT_H

#include <stdio.h>
#include <stdlib.h>


double** memory_allocation_for_storage_arrays(int sim_time, double dt);

void destroy_arrays(double** array);

#endif