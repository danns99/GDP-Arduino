#include "numerical_ode_solvers.h"


/*
 Calculates and returns the derivative of x with respect to time for the
 state-space system using a fourth-order Runge-Kutta scheme.
 */
double* xdot_solve(double A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
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


double* xdot_solve_backward_euler(
                   double A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                   double B[STATE_SPACE_MATRIX_SIZE],
                   double x[STATE_SPACE_MATRIX_SIZE],
                   double u[STATE_SPACE_MATRIX_SIZE],
                   double dt){
    //int i;
    //double x_fixed;
    //double N;
    double x_old[2];
    double inv_det_A;
    double inv_A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE];
    double B_times_u[STATE_SPACE_MATRIX_SIZE];

    x_old[0] = x[0];
    x_old[1] = x[1];

    inv_det_A = 1.0 / (((1-A[0][0])*dt)*((1-A[1][1])*dt) - ((-A[0][1]*dt)*(-A[1][0]*dt)));
    inv_A[0][0] = inv_det_A * (1-(A[1][1]*dt));
    inv_A[0][1] = inv_det_A * (A[0][1]*dt);
    inv_A[1][0] = inv_det_A * (A[1][0]*dt);
    inv_A[1][1] = inv_det_A * (1-(A[0][0]*dt));

    B_times_u[0] = x_old[0] + ((B[0]*0 + B[1]*u[0])*dt);
    B_times_u[1] = x_old[1] + ((B[0]*0 + B[1]*u[0])*dt);

    x[0] = inv_A[0][0]*B_times_u[0] + inv_A[0][1]*B_times_u[1];
    x[1] = inv_A[1][0]*B_times_u[0] + inv_A[1][1]*B_times_u[1];

    /*
    for(i=0; i<STATE_SPACE_MATRIX_SIZE; i++){
        N=0;
        // Use Euler's method to calculate start point 
        x_fixed = x[i];
        x_dot[i] = state_space_function(i, 0.0, A, B, x, u);
        x[i] = x[i] + x_dot[i]*dt; 
        while(N<15){
            x_dot[i] = state_space_function(i, 0.0, A, B, x, u);
            x[i] = x_fixed + x_dot[i]*dt;
            N++;
            //printf("x is: %lf\n", x[i]);
        }
    }
    */
    return x;
}


double* xdot_solve_mod(double A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                       double B[STATE_SPACE_MATRIX_SIZE],
                       double x[STATE_SPACE_MATRIX_SIZE],
                       double u[STATE_SPACE_MATRIX_SIZE],
                       double dt,
                       double q_out_of_target_aircraft, double *error_prior){
    int i;
    double x_old;
    double K1;
    double K2;
    double K3;
    double K4;
    double x_dot[STATE_SPACE_MATRIX_SIZE];

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
            u[0] = PD_controller(u_from_error_sum, error_prior, dt);
        }
        /* K2 = dt*(A*(x+K1/2)+B*u) */
        K2 = dt*state_space_function(i, K1/2.0, A, B, x, u);
        if (i==2){
            /* Get the control input for the modified Scout */
            /* Calculate the error between the pitch rate of the target aircraft
            and the modified Scout */
            u_from_error_sum = error_signal(q_out_of_target_aircraft, K2);
            /* Calculate the input to the modified Scout using the PID */
            u[0] = PD_controller(u_from_error_sum, error_prior, dt);
        }
        /* K3 = dt*(A*(x+K2/2)+B*u) */
        K3 = dt*state_space_function(i, K2/2.0, A, B, x, u);
        if (i==2){
            /* Get the control input for the modified Scout */
            /* Calculate the error between the pitch rate of the target aircraft
            and the modified Scout */
            u_from_error_sum = error_signal(q_out_of_target_aircraft, K3);
            /* Calculate the input to the modified Scout using the PID */
            u[0] = PD_controller(u_from_error_sum, error_prior, dt);
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
