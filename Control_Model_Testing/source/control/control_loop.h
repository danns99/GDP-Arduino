#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include "..\\state_space_equations\\numerical_ode_solvers.h"
#include <stdio.h>


int run_control_loop(double dt, double *u, double *u_into_modified_scout,
                     double A_sc[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                     double *B_sc, double *x_sc,
                     double A_t[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                     double *B_t, double *x_t, double* x_sc_mod,
                     double *integral);

#endif
