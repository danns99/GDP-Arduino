#ifndef BACKWARD_EULER_NEWTON_H
#define BACKWARD_EULER_NEWTON_H

#include <math.h>
#include "forward_euler.h"
#include "..//..//control//PID_controller.h"
#include "..//..//control//elevator_saturation.h"


double f_1_for_newton(double A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                      double B[STATE_SPACE_MATRIX_SIZE],
                      double x_1, double x_2, double x_fixed,
                      double u,
                      double dt);

double f_2_for_newton(double A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                      double B[STATE_SPACE_MATRIX_SIZE],
                      double x_1, double x_2, double x_fixed,
                      double u,
                      double dt);

double* xdot_solve_backward_euler_newton(
                    double A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                    double B[STATE_SPACE_MATRIX_SIZE],
                    double x[STATE_SPACE_MATRIX_SIZE],
                    double u[STATE_SPACE_MATRIX_SIZE],
                    double dt,
                    double q_out_of_target_aircraft, double *error_prior);

#endif
