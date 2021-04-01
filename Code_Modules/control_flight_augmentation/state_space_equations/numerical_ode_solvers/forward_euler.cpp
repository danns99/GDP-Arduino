#include "forward_euler.h"


/*
 Calculates and returns the derivative of x with respect to time for the
 state-space system using the forward Euler method.
 */
double* xdot_solve_forward_euler(
                    double A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                    double B[STATE_SPACE_MATRIX_SIZE],
                    double x[STATE_SPACE_MATRIX_SIZE],
                    double u[STATE_SPACE_MATRIX_SIZE],
                    double dt){
    int i;
    double x_old;
    double x_dot[STATE_SPACE_MATRIX_SIZE];

    for (i=0; i<STATE_SPACE_MATRIX_SIZE; i++){
        x_old = x[i];
        x_dot[i] = state_space_function(i, 0.0, A, B, x, u);
        x[i] = x_old + x_dot[i]*dt;
    }
    return x;
}
