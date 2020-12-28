#include "sim_init_full_longitudinal_model.h"


int init_sim(void){
    /* Simulation time settings */
    double sim_time = 5;
    double input_time = 1;
    double dt = 0.0001;
    int steps = 0;

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
    double u_into_modified_scout[4] = {0};
    double error_prior = 0;

    /* Aircraft state-space A and B matrices */
    /* Scout matrices */
    double A_sc[4][4];
    double B_sc[4];
    /* Target aircraft matrices */
    double A_t[4][4];
    double B_t[4];

    /* Get the aircraft state matrices from text files */
    get_aircraft_state_space_matrices(A_sc, B_sc, A_t, B_t);

    run_sim(sim_time, input_time, dt, steps, u, u_old, x_sc, x_t, x_sc_mod,
            x_sc_old, x_t_old, x_sc_mod_old, xdot_sc_store, xdot_t_store,
            xdot_sc_mod_store, xdot_sc, xdot_t, xdot_sc_mod, x_sc_store,
            x_t_store, x_sc_mod_store, u_into_modified_scout, error_prior, A_sc,
            B_sc, A_t, B_t);

    return 0;
}
