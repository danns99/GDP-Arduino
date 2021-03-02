#include "numerical_ode_solvers.h"


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


/*
 Function used in Newton's method.
 */
double f_1_for_newton(double A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                      double B[STATE_SPACE_MATRIX_SIZE],
                      double x_1, double x_2, double x_fixed,
                      double u,
                      double dt){
    return x_1 - x_fixed - dt*(A[0][0]*x_1 + A[0][1]*x_2 + B[0]*u);
}


/*
 Function used in Newton's method.
 */
double f_2_for_newton(double A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                      double B[STATE_SPACE_MATRIX_SIZE],
                      double x_1, double x_2, double x_fixed,
                      double u,
                      double dt){
    return x_2 - x_fixed - dt*(A[1][0]*x_1 + A[1][1]*x_2 + B[1]*u);
}


/*
 Calculates and returns the derivative of x with respect to time for the
 state-space system using the backwards Euler method; which is solved with
 Newton's method.
 */
double* xdot_solve_backward_euler_newton(
                    double A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                    double B[STATE_SPACE_MATRIX_SIZE],
                    double x[STATE_SPACE_MATRIX_SIZE],
                    double u[STATE_SPACE_MATRIX_SIZE],
                    double dt,
                    double q_out_of_target_aircraft, double *error_prior){
    int N = 0;
    int max_iter = 100;
    int tol = 1e-6;
    double u_from_error_sum;
    double u_temp;
    double df1_dx1;
    double df1_dx2;
    double df2_dx1;
    double df2_dx2;
    double f_1;
    double f_2;
    double inv_det_J;
    double h = 1e-6;
    double x_fixed[2];
    double u_old[2];
    double inv_J[2][2];

    x_fixed[0] = x[0];
    x_fixed[1] = x[1];

    /* Use the forward Euler method for the initial guess */
    x = xdot_solve_forward_euler(A, B, x, u, dt);

    while(N < max_iter){
        u_old[0] = u[0];
        /* Get the control input for the modified Scout */
        /* Calculate the error between the pitch rate of the target aircraft
        and the modified Scout */
        u_from_error_sum = error_signal(q_out_of_target_aircraft, x[1]);
        /* Calculate the input to the modified Scout using the PID */
        u_temp= PD_controller(u_from_error_sum, error_prior, dt);
        u[0] = u_old[0] + u_temp*dt;

        /* Calculate the terms in the Jacobian */
        df1_dx1 = ((f_1_for_newton(A, B, x[0]+h, x[1], x_fixed[0], u[0], dt) -
                   f_1_for_newton(A, B, x[0]-h, x[1], x_fixed[0], u[0], dt)) /
                   (2*h));
        df1_dx2 = ((f_1_for_newton(A, B, x[0], x[1]+h, x_fixed[0], u[0], dt) -
                   f_1_for_newton(A, B, x[0], x[1]-h, x_fixed[0], u[0], dt)) /
                   (2*h));
        df2_dx1 = ((f_2_for_newton(A, B, x[0]+h, x[1], x_fixed[1], u[0], dt) -
                   f_2_for_newton(A, B, x[0]-h, x[1], x_fixed[1], u[0], dt)) /
                   (2*h));
        df2_dx2 = ((f_2_for_newton(A, B, x[0], x[1]+h, x_fixed[1], u[0], dt) -
                   f_2_for_newton(A, B, x[0], x[1]-h, x_fixed[1], u[0], dt)) /
                   (2*h));

        /* Calculate the inverse of the Jacobian */
        /* Calculate 1 over the determinant of the Jacobian */
        inv_det_J = 1 / (df1_dx1*df2_dx2 - df1_dx2*df2_dx1);
        /* Calculate the terms within the inverse of the Jacobian */
        inv_J[0][0] = inv_det_J * df2_dx2;
        inv_J[0][1] = inv_det_J * -df1_dx2;
        inv_J[1][0] = inv_det_J * -df2_dx1;
        inv_J[1][1] = inv_det_J * df1_dx1;
        
        /* Calculate f_1 and f_2 in vector F used in Newton's method */
        f_1 = f_1_for_newton(A, B, x[0], x[1], x_fixed[0], u[0], dt);
        f_2 = f_2_for_newton(A, B, x[0], x[1], x_fixed[1], u[0], dt);

        /* Update the states */
        x[0] = x[0] - (inv_J[0][0]*f_1 + inv_J[0][1]*f_2);
        x[1] = x[1] - (inv_J[1][0]*f_1 + inv_J[1][1]*f_2);

        /* Check if the input into the modified Scout has converged */
        if(abs(u_old[0] - u[0]) <= tol){
            break;
        }

        /* Increment the iteration counter */
        N += 1;
    }

    return x;
}
