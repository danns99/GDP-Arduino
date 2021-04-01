#include "runge_kutta.h"


/*
 Calculates and returns the derivative of x with respect to time for the
 state-space system using a fourth-order Runge-Kutta scheme.
 */
double* xdot_solve_runge_kutta(double A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                   double B[STATE_SPACE_MATRIX_SIZE],
                   double x[STATE_SPACE_MATRIX_SIZE],
                   double u[STATE_SPACE_MATRIX_SIZE],
                   double dt){
    int i;
    double x_old;
    double K1;
    double K2;
    double K3;
    double K4;
    double x_dot[STATE_SPACE_MATRIX_SIZE];
    
    /* Solve the state-space equations */
    for (i=0; i<STATE_SPACE_MATRIX_SIZE; i++){
        x_old = x[i];
        /* K1 = dt*(A*x+B*u) */
        K1 = dt*state_space_function(i, 0.0, A, B, x, u);
        /* K2 = dt*(A*(x+K1/2)+B*u) */
        K2 = dt*state_space_function(i, K1/2.0, A, B, x, u);
        /* K3 = dt*(A*(x+K2/2)+B*u) */
        K3 = dt*state_space_function(i, K2/2.0, A, B, x, u);
        /* K4 = dt*(A*(x+K3)+B*u) */
        K4 = dt*state_space_function(i, K3, A, B, x, u);

        /* Calculate the derivative of x with respect to time */
        x_dot[i] = (K1 + (2.0*K2) + (2.0*K3) + K4)/6.0;

        /* Update the state */
        x[i] = x_old + x_dot[i];
    }

    return x;
}
