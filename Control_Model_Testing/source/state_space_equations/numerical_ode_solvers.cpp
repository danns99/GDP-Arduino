#include "numerical_ode_solvers.h"


/*
Calculates and returns the derivative of x with respect to time for the
state-space system using a fourth-order Runge-Kutta scheme.
*/
double* xdot_solve(double* x_dot,
                   double A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                   double B[STATE_SPACE_MATRIX_SIZE],
                   double x[STATE_SPACE_MATRIX_SIZE],
                   double u[STATE_SPACE_MATRIX_SIZE],
                   double dt){
    int i;
    double error;
    double x_old;
    double K1;
    double K2;
    double K3;
    double K4;
    
    /* Solve the state-space equations */
    for (i=0; i<STATE_SPACE_MATRIX_SIZE; i++){
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
        x_old = x[i];

        /* Update the state */
        x[i] = x_old + x_dot[i];
    }

    /* Use Euler's method to calculate start point */
    // for(i=0; i<4; i++){
    //     x_old = x[i];
    //     B_times_u = B[i]*u[0];
    //     x_dot[i] = state_space_function(i, 0.0, A, B, x, u);
    //     x[i] = x_old + x_dot[i]*dt; 
    //     while(abs(error) > 0.01){
    //         B_times_u = B[i]*u[0];
    //         x_dot[i] = state_space_function(i, 0.0, A, B, x, u);
    //         x[i] = x_old + x_dot[i]*dt;
    //         x_old = x[i];
    //         error = x[i] - x_old;
    //     }
    // }
    return x;
}


double* xdot_solve_mod(double* x_dot,
                       double A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                       double B[STATE_SPACE_MATRIX_SIZE],
                       double x[STATE_SPACE_MATRIX_SIZE],
                       double u[STATE_SPACE_MATRIX_SIZE],
                       double dt,
                       double q_out_of_target_aircraft, double *error_prior){
    int i;
    double error;
    double x_old;
    double K1;
    double K2;
    double K3;
    double K4;

    double u_from_error_sum;
    
    /* Solve the state-space equations */
    for (i=0; i<STATE_SPACE_MATRIX_SIZE; i++){
        /* K1 = dt*(A*x+B*u) */
        K1 = dt*state_space_function(i, 0.0, A, B, x, u);
        if (i==2){
            /* Get the control input for the modified Scout */
            /* Calculate the error between the pitch rate of the target aircraft
            and the modified Scout */
            u_from_error_sum = error_signal(q_out_of_target_aircraft, K1);
            /* Calculate the input to the modified Scout using the PID */
            u[2] = PID_controller(u_from_error_sum, error_prior, dt);
        }
        /* K2 = dt*(A*(x+K1/2)+B*u) */
        K2 = dt*state_space_function(i, K1/2.0, A, B, x, u);
        if (i==2){
            /* Get the control input for the modified Scout */
            /* Calculate the error between the pitch rate of the target aircraft
            and the modified Scout */
            u_from_error_sum = error_signal(q_out_of_target_aircraft, K2);
            /* Calculate the input to the modified Scout using the PID */
            u[2] = PID_controller(u_from_error_sum, error_prior, dt);
        }
        /* K3 = dt*(A*(x+K2/2)+B*u) */
        K3 = dt*state_space_function(i, K2/2.0, A, B, x, u);
        if (i==2){
            /* Get the control input for the modified Scout */
            /* Calculate the error between the pitch rate of the target aircraft
            and the modified Scout */
            u_from_error_sum = error_signal(q_out_of_target_aircraft, K3);
            /* Calculate the input to the modified Scout using the PID */
            u[2] = PID_controller(u_from_error_sum, error_prior, dt);
        }
        /* K4 = dt*(A*(x+K3)+B*u) */
        K4 = dt*state_space_function(i, K3, A, B, x, u);

        /* Calculate the derivative of x with respect to time */
        x_dot[i] = (K1 + (2.0*K2) + (2.0*K3) + K4)/6.0;
        x_old = x[i];

        /* Update the state */
        x[i] = x_old + x_dot[i];
    }
    return x;
}
