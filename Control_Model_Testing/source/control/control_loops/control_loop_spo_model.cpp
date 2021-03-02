#include "control_loop_spo_model.h"


/*
 Runs the main control loop.
 */
int run_control_loop_spo(double dt, double *u, double *u_into_modified_scout,
                     double A_sc[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                     double *B_sc, double *x_sc,
                     double A_t[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                     double *B_t, double *x_t, double* x_sc_mod,
                     double *error_prior){
    double q_out_of_target_aircraft;

    /* Solve the state-space equations for the target aircraft and the
    Scout */
    xdot_solve_backward_euler(A_t, B_t, x_t, u, dt);
    xdot_solve_backward_euler(A_sc, B_sc, x_sc, u, dt);

    /* Get the control input for the modified Scout */
    /* Get the pitch rate output of the target aircraft */
    q_out_of_target_aircraft = x_t[1];

    /* Solve the state-space equations for the modified scout */
    xdot_solve_backward_euler_newton(A_sc, B_sc, x_sc_mod,
                                     u_into_modified_scout, dt,
                                     q_out_of_target_aircraft, error_prior);
    return 0;
}
