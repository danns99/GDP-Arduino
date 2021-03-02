#ifndef NUMERICAL_ODE_SOLVERS_H
#define NUMERICAL_ODE_SOLVERS_H

#include <math.h>
#include "state_space_equation.h"
#include "..\\control\\PID_controller.h"


double* xdot_solve(double A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                   double B[STATE_SPACE_MATRIX_SIZE],
                   double x[STATE_SPACE_MATRIX_SIZE],
                   double u[STATE_SPACE_MATRIX_SIZE],
                   double dt);

double* xdot_solve_backward_euler(
                   double A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                   double B[STATE_SPACE_MATRIX_SIZE],
                   double x[STATE_SPACE_MATRIX_SIZE],
                   double u[STATE_SPACE_MATRIX_SIZE],
                   double dt);

double* xdot_solve_backward_euler_newton(
                    double A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                    double B[STATE_SPACE_MATRIX_SIZE],
                    double x[STATE_SPACE_MATRIX_SIZE],
                    double u[STATE_SPACE_MATRIX_SIZE],
                    double dt,
                    double q_out_of_target_aircraft, double *error_prior);

double* xdot_solve_mod(double A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                       double B[STATE_SPACE_MATRIX_SIZE],
                       double x[STATE_SPACE_MATRIX_SIZE],
                       double u[STATE_SPACE_MATRIX_SIZE],
                       double dt,
                       double q_out_of_target_aircraft, double *error_prior);

#endif
