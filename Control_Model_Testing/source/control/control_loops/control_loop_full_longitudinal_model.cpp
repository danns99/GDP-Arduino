#include "control_loop_full_longitudinal_model.h"


/*
 Runs the main control loop.
 */
int run_control_loop_full(double dt, double *u, double *u_into_modified_scout,
                     double A_sc[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                     double *B_sc, double *x_sc,
                     double A_t[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                     double *B_t, double *x_t, double* x_sc_mod,
                     double *error_prior){
    double q_out_of_target_aircraft;
    double current_error;
    double u_from_error_sum;
    double u_temp;
    
    /* Solve the state-space equations for the target aircraft and the
    Scout */
    xdot_solve_runge_kutta(A_t, B_t, x_t, u, dt);
    xdot_solve_runge_kutta(A_sc, B_sc, x_sc, u, dt);

    /* Get the control input for the modified Scout */
    /* Get the pitch rate output of the target aircraft */
    q_out_of_target_aircraft = x_t[2];
    /* Get the current error as the pitch rate output of the modified Scout
    aircraft */
    current_error = x_sc_mod[2];
    /* Calculate the error between the pitch rate of the target aircraft and
    the modified Scout */
    u_from_error_sum = error_signal(q_out_of_target_aircraft, current_error);
    /* Calculate the input to the modified Scout using the PD controller */
    u_into_modified_scout[0] = PD_controller(u_from_error_sum, error_prior, dt);

    /* Limit the control input to the range of elevator angles possible
    on the Scout */
    u_temp = u_into_modified_scout[0];
    u_into_modified_scout[0] = saturate_elevator(u_temp);

    /* Solve the state-space equations for the modified scout */
    xdot_solve_runge_kutta(A_sc, B_sc, x_sc_mod, u_into_modified_scout, dt);

    return 0;
}
