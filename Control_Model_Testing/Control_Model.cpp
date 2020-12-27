#include <stdio.h>
#include <stdlib.h>
#include <math.h>


/*
Calculates and returns the error between the output of the target aircraft and
the output of the modified Sherwood Scout.
*/
double error_signal(double target_input, double error_from_system_output){
    return (target_input - error_from_system_output);
}


/*
Calculates and returns the proportional (P) integral (I) and derivative (D)
gains using the current error, the previous error and the timestep size. The PID
gains were determined using MATLAB's response optimizer.
*/
double PID_controller(double error, double *error_prior, double iteration_time){
    double proportional_term;
    double integral_term;
    double derivative_term;
    double K_p = -98.687493079853600;
    double K_i = -19.396423994327110;
    double K_d = -0.018045698057602;

    proportional_term = K_p*error;
    integral_term += K_i*(error*iteration_time);
    derivative_term = K_d*((error - (*error_prior)) / iteration_time);

    *error_prior = error;
    return (proportional_term + integral_term + derivative_term);
}


/*
Calculates and returns the state-space equation solution
*/
double state_space_function(int i, double y_option, double A[4][4], double B[4],
                            double x[4], double u[4]){
    double B_times_u;

    B_times_u = B[i]*u[0];
    return A[i][0]*(x[0]+y_option) + A[i][1]*(x[1]+y_option) +
           A[i][2]*(x[2]+y_option) + A[i][3]*(x[3]+y_option) +
           B_times_u;
}


/*
Calculates and returns the derivative of x with respect to time for the
state-space system using a fourth-order Runge-Kutta scheme.
*/
double* xdot_solve_rk(double* x_dot, double A[4][4], double B[4],
                   double x[4], double u[4], double dt){
    int i;
    double error;
    double x_old;
    double K1;
    double K2;
    double K3;
    double K4;
    
    /* Solve the state-space equations */
    for (i=0; i<4; i++){
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

double* xdot_solve_mod(double* x_dot, double A[4][4], double B[4],
                       double x[4], double u[4], double dt,
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
    for (i=0; i<4; i++){
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
/*
Returns a storage array for the state values which are solved during the
simulation
*/
double** memory_allocation_for_storage_arrays(int sim_time, double dt){
    int i;
    double **x_store;

    /* Allocate memory for the 2D arrays */
    x_store = (double **) malloc(sim_time*1/dt*sizeof(double *));
    
    /* If memory could not be allocated */
    if(x_store == NULL){
        printf("Error: out of memory.\n");
    }

    /* Allocate memory for each sub array */
    for(i=0; i<5*1/dt; i++){
        x_store[i] = (double *) malloc(4*sizeof(double));
        /* If memory could not be allocated */
        if(x_store[i] == NULL){
            printf("Error: out of memory.\n");
        }
    }
    return x_store;
}


/*
Frees the memory of used by the storage arrays.
*/
void destroy_arrays(double** array){
    free(*array);
    free(array);
}


/*
Runs the simulation
*/
int main(void) {
    int i;
    FILE *f;

    /* Simulation time settings*/
    int sim_time = 5;
    double dt = 0.0001;
    int steps = 0;

    /* Aircraft state-space A and B matrices */
    /* Scout matrices */
    double A_sc[4][4] = {
        {-0.0787229113327511,0.0794243229686747,0,-9.81000000000000},
        {-0.353674533020279,-3.33670195088189,53.1386667274010,0},
        {0.00876928712067767,-1.15098695271769,-5.34685805582989,0},
        {0,0,1,0}};
    double B_sc[4] = {0.688453509731723,-12.6068373237743,-35.8857538372364,0};
    /* Target aircraft matrices */
    double A_t[4][4] = {
        {0.00719077960641651,0.00456992967091996,0,-9.81000000000000},
        {-0.0687424646665509,-0.295319442940517,177.255749314106,0},
        {0.00172978605332707,-0.0104482212700093,-0.448358567032906,0},
        {0,0,1,0}};
    double B_t[4] = {1.04080857871611, -6.29389106785740, -4.88848338612147, 0};

    /* Control vectors */
    double u[4] = {0};
    double u_old=0;

    /* x vectors */
    double x_sc[4] = {0};
    double x_t[4] = {0};
    double x_sc_mod[4] = {0};
    double x_sc_old;
    double x_t_old;
    double x_sc_mod_old;

    /* x dot vectors */
    /* For storing data at the end of each timestep */
    double xdot_sc_store[4] = {0};
    double xdot_t_store[4] = {0};
    double xdot_sc_mod_store[4] = {0};
    /* For use during the timestep */
    double *xdot_sc;
    double *xdot_t;
    double *xdot_sc_mod;

    /* Arrays for storing the aircraft data for all timesteps */
    double** x_sc_store = memory_allocation_for_storage_arrays(sim_time, dt); 
    double** x_t_store = memory_allocation_for_storage_arrays(sim_time, dt);
    double** x_sc_mod_store = memory_allocation_for_storage_arrays(sim_time,
                                                                   dt);
    /* Variables for the modified Scout control */
    double q_out_of_target_aircraft;
    double u_from_error_sum;
    double u_into_modified_scout[4] = {0};
    double current_error = 0;
    double error_prior = 0;

    /* Main time loop */
    while(steps < sim_time*1/dt){
        /* Select time to apply step input */
        if(steps == 99){
            printf("here\n");
            u_old = u[0];
            u[0] = u_old + 0.1;
        }

        /* Solve the state-space equations for the target aircraft and the
        Scout */
        xdot_solve(xdot_t_store, A_t, B_t, x_t, u, dt);
        xdot_solve(xdot_sc_store, A_sc, B_sc, x_sc, u, dt);

        /* Get the control input for the modified Scout */
        /* Get the pitch rate output of the target aircraft */
        q_out_of_target_aircraft = x_t[2];
        /* Get the current error as the pitch rate output of the modified Scout
        aircraft */
        current_error = x_sc_mod[2];
        /* Calculate the error between the pitch rate of the target aircraft and
        the modified Scout */
        u_from_error_sum = error_signal(q_out_of_target_aircraft,
                                        current_error);
        /* Calculate the input to the modified Scout using the PID */
        u_into_modified_scout[0] = PID_controller(u_from_error_sum,
                                                  &error_prior, dt);

        /* Solve the state-space equations for the modified scout */
        xdot_solve(xdot_sc_mod_store, A_sc, B_sc, x_sc_mod,
                       u_into_modified_scout, dt);
        // xdot_solve_mod(xdot_sc_mod_store, A_sc, B_sc, x_sc_mod,
        //                u_into_modified_scout, dt, q_out_of_target_aircraft,
        //                &error_prior);

        /* Store data for the states of the target, Scout and modified Scout
        aircraft at the end of the timestep */
        for (i=0; i<4; i++){
            x_sc_store[steps][i] = x_sc[i];
            x_t_store[steps][i] = x_t[i];
            x_sc_mod_store[steps][i] = x_sc_mod[i];
        }
        steps += 1;
    }

    printf("%f\n", error_prior);

    /* Write the simulation data to a text file at the end of the simulation */
    f=fopen("test_data.txt","w");
    for(i=0; i<steps; i++){
        fprintf(f, "%f %f %f %f %f %f %f %f %f %f %f %f %f \n",
                i*dt, x_sc_store[i][0], x_sc_store[i][1], x_sc_store[i][2],
                x_sc_store[i][3], x_t_store[i][0], x_t_store[i][1],
                x_t_store[i][2], x_t_store[i][3], x_sc_mod_store[i][0],
                x_sc_mod_store[i][1], x_sc_mod_store[i][2],
                x_sc_mod_store[i][3]);
    }

    /* Free memory used by the storage arrays */
    destroy_arrays(x_sc_store);
    destroy_arrays(x_t_store);
    destroy_arrays(x_sc_mod_store);

    return 0;
}
