#ifndef NUMERICAL_ODE_SOLVERS_H
#define NUMERICAL_ODE_SOLVERS_H

#include "state_space_equation.h"
#include "..\\control\\PID_controller.h"


double* xdot_solve(double* x_dot,
                   double A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                   double B[STATE_SPACE_MATRIX_SIZE],
                   double x[STATE_SPACE_MATRIX_SIZE],
                   double u[STATE_SPACE_MATRIX_SIZE],
                   double dt);

double* xdot_solve_backward_euler(double* x_dot,
                   double A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                   double B[STATE_SPACE_MATRIX_SIZE],
                   double x[STATE_SPACE_MATRIX_SIZE],
                   double u[STATE_SPACE_MATRIX_SIZE],
                   double dt);

#endif
