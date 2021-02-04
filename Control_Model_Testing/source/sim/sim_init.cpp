#include "sim_init.h"


/*
 Initialises variables used by the simulation and calls the function run_sim to
 run the simulation.
 */
int init_sim(void){
    /* Variables for simulation settings */
    int steps = 0;
    double sim_time = 0;
    double input_time = 0;
    double dt = 0;
    char aircraft_data_folder_dir[100];
    /* Read in the simulation settings */
    read_sim_settings(&sim_time, &dt, &input_time, aircraft_data_folder_dir);

    /* Control vectors */
    double u[STATE_SPACE_MATRIX_SIZE] = {0};
    double u_old=0;

    /* x vectors */
    double x_sc[STATE_SPACE_MATRIX_SIZE] = {0};
    double x_t[STATE_SPACE_MATRIX_SIZE] = {0};
    double x_sc_mod[STATE_SPACE_MATRIX_SIZE] = {0};

    /* x dot vectors */
    /* For storing data at the end of each timestep */
    double xdot_sc_store[STATE_SPACE_MATRIX_SIZE] = {0};
    double xdot_t_store[STATE_SPACE_MATRIX_SIZE] = {0};
    double xdot_sc_mod_store[STATE_SPACE_MATRIX_SIZE] = {0};

    /* Arrays for storing the aircraft data for all timesteps */
    double** x_sc_store = memory_allocation_for_storage_arrays(sim_time, dt); 
    double** x_t_store = memory_allocation_for_storage_arrays(sim_time, dt);
    double** x_sc_mod_store = memory_allocation_for_storage_arrays(sim_time,
                                                                   dt);
    /* Arrays for storing inputs to the aircraft for all timesteps */
    double* u_store = memory_allocation_for_1_d_arrays(sim_time, dt);
    double* u_sc_mod_store = memory_allocation_for_1_d_arrays(sim_time, dt);

    /* Variables for the modified Scout control */
    double u_into_modified_scout[STATE_SPACE_MATRIX_SIZE] = {0};
    double integral = 0;

    /* Aircraft state-space A and B matrices */
    /* Scout matrices */
    double A_sc[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE];
    double B_sc[STATE_SPACE_MATRIX_SIZE];
    /* Target aircraft matrices */
    double A_t[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE];
    double B_t[STATE_SPACE_MATRIX_SIZE];

    /* Get the aircraft state matrices from text files */
    get_aircraft_state_space_matrices(A_sc, B_sc, A_t, B_t,
                                      aircraft_data_folder_dir);

    /* Run the simulation */
    run_sim(sim_time, input_time, dt, steps, u, u_old, x_sc, x_t, x_sc_mod,
            xdot_sc_store, xdot_t_store, xdot_sc_mod_store,  x_sc_store,
            x_t_store, x_sc_mod_store, u_store, u_sc_mod_store,
            u_into_modified_scout, &integral, A_sc, B_sc, A_t, B_t);

    return 0;
}
