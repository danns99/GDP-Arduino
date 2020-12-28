#ifndef NUMERICAL_ODE_SOLVERS_H
#define NUMERICAL_ODE_SOLVERS_H

#include "state_space_equation.h"
#include "..\\controller\\PID_controller.h"


double* xdot_solve(double* x_dot, double A[4][4], double B[4],
                   double x[4], double u[4], double dt);

#endif
