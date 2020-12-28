#ifndef FILE_IO_H
#define FILE_IO_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


int read_state_space_matrices_from_file(double A[4][4], double B[4],
                                        char* file_dir);

int get_aircraft_state_space_matrices(void);

#endif
