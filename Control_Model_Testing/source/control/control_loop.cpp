#include "control_loop.h"


/*
 Runs the main control loop.
 */
int run_control_loop(double dt, double *u, double *u_into_modified_scout,
                     double A_sc[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                     double *B_sc, double *x_sc,
                     double A_t[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                     double *B_t, double *x_t, double* x_sc_mod,
                     double *xdot_t_store, double *xdot_sc_store,
                     double *xdot_sc_mod_store, double *error_prior){
    double q_out_of_target_aircraft;
    double current_error;
    double u_from_error_sum;
    
    /* Solve the state-space equations for the target aircraft and the
    Scout */
    xdot_solve(xdot_t_store, A_t, B_t, x_t, u, dt);
    xdot_solve(xdot_sc_store, A_sc, B_sc, x_sc, u, dt);

    /* Get the control input for the modified Scout */
    /* Get the pitch rate output of the target aircraft */
    if(STATE_SPACE_MATRIX_SIZE == 4){
        q_out_of_target_aircraft = x_t[2];
    }
    else if(STATE_SPACE_MATRIX_SIZE == 2){
        q_out_of_target_aircraft = x_t[1];
    }
    
    /* Get the current error as the pitch rate output of the modified Scout
    aircraft */
    if(STATE_SPACE_MATRIX_SIZE == 4){
        current_error = x_sc_mod[2];
    }
    else if(STATE_SPACE_MATRIX_SIZE == 2){
        current_error = x_sc_mod[1];
    }
    /* Calculate the error between the pitch rate of the target aircraft and
    the modified Scout */
    u_from_error_sum = error_signal(q_out_of_target_aircraft,
                                    current_error);

    /* Calculate the input to the modified Scout using the PID */
    u_into_modified_scout[0] = PD_controller(u_from_error_sum,
                                                error_prior, dt);

    /* Solve the state-space equations for the modified scout */
    xdot_solve(xdot_sc_mod_store, A_sc, B_sc, x_sc_mod,
                     u_into_modified_scout, dt);
    //xdot_solve_backward_euler(xdot_sc_mod_store, A_sc, B_sc, x_sc_mod,
    //                          u_into_modified_scout, dt);
    // xdot_solve_mod(xdot_sc_mod_store, A_sc, B_sc, x_sc_mod,
    //                u_into_modified_scout, dt, q_out_of_target_aircraft,
    //                &error_prior);
    return 0;
}
