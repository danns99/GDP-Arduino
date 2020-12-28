#ifndef CONTROL_MODEL_TESTING_H
#define CONTROL_MODEL_TESTING_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "util\\file_io.h"
#include "util\\memory_management.h"


double error_signal(double target_input, double error_from_system_output);

double state_space_function(int i, double y_option, double A[4][4], double B[4],
                            double x[4], double u[4]);

double* xdot_solve(double* x_dot, double A[4][4], double B[4],
                   double x[4], double u[4], double dt);

double* xdot_solve_mod(double* x_dot, double A[4][4], double B[4],
                       double x[4], double u[4], double dt,
                       double q_out_of_target_aircraft, double *error_prior);        

#endif
