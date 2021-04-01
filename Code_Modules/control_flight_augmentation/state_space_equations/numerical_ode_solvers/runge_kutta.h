#ifndef RUNGE_KUTTA_H
#define RUNGE_KUTTA_H

#include "..//state_space_equation.h"


double* xdot_solve_runge_kutta(double A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                   double B[STATE_SPACE_MATRIX_SIZE],
                   double x[STATE_SPACE_MATRIX_SIZE],
                   double u[STATE_SPACE_MATRIX_SIZE],
                   double dt);

#endif
