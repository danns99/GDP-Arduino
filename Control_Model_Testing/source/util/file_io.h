#ifndef FILE_IO_H
#define FILE_IO_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


int read_state_space_matrices_from_file(double A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                                        double B[STATE_SPACE_MATRIX_SIZE],
                                        char* file_dir);

int get_aircraft_state_space_matrices(double A_sc[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                                      double *B_sc,
                                      double A_t[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                                      double *B_t);

int write_sim_data_to_file(int steps, double dt, double **x_sc_store,
                           double **x_t_store, double **x_sc_mod_store);

int store_timestep_data(int steps, double *x_sc, double *x_t, double *x_sc_mod,
                        double **x_sc_store, double **x_t_store,
                        double **x_sc_mod_store);

#endif
