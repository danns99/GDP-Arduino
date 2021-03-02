#include "backward_euler_analytic.h"


/*
 Calculates and returns the derivative of x with respect to time for the
 state-space system using an analytical expression of the backwards Euler
 method.
 */
double* xdot_solve_backward_euler(
                   double A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                   double B[STATE_SPACE_MATRIX_SIZE],
                   double x[STATE_SPACE_MATRIX_SIZE],
                   double u[STATE_SPACE_MATRIX_SIZE],
                   double dt){
    double inv_det_A;
    double x_old[2];
    double inv_A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE];
    double B_times_u[STATE_SPACE_MATRIX_SIZE];

    x_old[0] = x[0];
    x_old[1] = x[1];

    inv_det_A = 1.0 / ((1-(A[0][0])*dt)*(1-(A[1][1])*dt) - ((-A[0][1]*dt)*(-A[1][0]*dt)));
    inv_A[0][0] = inv_det_A * (1-(A[1][1]*dt));
    inv_A[0][1] = inv_det_A * (A[0][1]*dt);
    inv_A[1][0] = inv_det_A * (A[1][0]*dt);
    inv_A[1][1] = inv_det_A * (1-(A[0][0]*dt));

    B_times_u[0] = x_old[0] + ((B[0]*u[0])*dt);
    B_times_u[1] = x_old[1] + ((B[1]*u[0])*dt);

    x[0] = inv_A[0][0]*B_times_u[0] + inv_A[0][1]*B_times_u[1];
    x[1] = inv_A[1][0]*B_times_u[0] + inv_A[1][1]*B_times_u[1];

    return x;
}
