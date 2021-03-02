#ifndef SIM_H
#define SIM_H

#define _USE_MATH_DEFINES

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "..\\util\\file_io.h"
#include "..\\util\\memory_management.h"
#include "sim_init.h"
#include "..\\control\\PID_controller.h"
#include "..\\state_space_equations\\state_space_equation.h"
#include "..\\control\\control_loops\\control_loop_full_longitudinal_model.h"
#include "..\\control\\control_loops\\control_loop_spo_model.h"

int run_sim(double sim_time, double input_time, double dt, int steps,
            double *u, double u_old, double *x_sc, double *x_t,
            double *x_sc_mod, double **x_sc_store, double **x_t_store,
            double **x_sc_mod_store, double *u_store, double *u_sc_mod_store,
            double *u_into_modified_scout, double *integral,
            double A_sc[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
            double *B_sc,
            double A_t[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
            double *B_t);

#endif
