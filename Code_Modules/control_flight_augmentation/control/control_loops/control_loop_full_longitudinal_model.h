#ifndef CONTROL_LOOP_FULL_LONGITUDINAL_MODEL_H
#define CONTROL_LOOP_FULL_LONGITUDINAL_MODEL_H

#include "..//..//state_space_equations//numerical_ode_solvers//runge_kutta.h"
#include "..//PID_controller.h"
#include "..//elevator_saturation.h"


int run_control_loop_full(double dt, double *u, double *u_into_modified_scout,
                     double A_sc[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                     double *B_sc, double *x_sc,
                     double A_t[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                     double *B_t, double *x_t, double* x_sc_mod,
                     double *integral);

#endif
