#include "sim.h"


/*
 Runs the simulation.
 */
int run_sim(double sim_time, double input_time, double dt, int steps,
            double *u, double u_old, double *x_sc, double *x_t,
            double *x_sc_mod, double **x_sc_store, double **x_t_store,
            double **x_sc_mod_store, double *u_store, double *u_sc_mod_store,
            double *u_into_modified_scout, double *integral,
            double A_sc[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
            double *B_sc,
            double A_t[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
            double *B_t){
    /* Main time loop */
    while(steps < sim_time*1/dt){
        /* Store the aircraft data from the timestep */
        store_timestep_data(steps, x_sc, x_t, x_sc_mod, u,
                            u_into_modified_scout, x_sc_store, x_t_store,
                            x_sc_mod_store, u_store, u_sc_mod_store);

        /* Select time to apply step input */
        float difference = (float) steps - (input_time)*1/(dt);
        float tol_difference = 0.001;
        if((-tol_difference <= difference) && (difference <= tol_difference)){
            u_old = u[0];
            u[0] = u_old + (15*(M_PI/180));
            printf("here\n");
        }

        /* Run the control loop */
        run_control_loop(dt, u, u_into_modified_scout,
                         A_sc, B_sc, x_sc,
                         A_t, B_t, x_t, x_sc_mod,
                         integral);

        /* Increment the step counter */
        steps += 1;
    }

    /* Write the simulation data to file */
    write_sim_data_to_file(steps, dt, x_sc_store, x_t_store, x_sc_mod_store,
                           u_store, u_sc_mod_store);

    /* Free memory used by the storage arrays */
    destroy_arrays(x_sc_store);
    destroy_arrays(x_t_store);
    destroy_arrays(x_sc_mod_store);

    return 0;
}
