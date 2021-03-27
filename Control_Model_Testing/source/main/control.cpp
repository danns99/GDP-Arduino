#include "control.h"


std::tuple<double, double> control_loop::init(){
    /* Read in the control model settings */
    read_sim_settings(&sim_time, &dt, &input_time, aircraft_data_folder_dir);

    /* Allocate memory for the storage arrays */
    x_sc_store = memory_allocation_for_x_vector_storage_arrays(sim_time, dt);
    x_t_store = memory_allocation_for_x_vector_storage_arrays(sim_time, dt);
    x_sc_mod_store = memory_allocation_for_x_vector_storage_arrays(sim_time, dt);
    u_store = memory_allocation_for_u_vector_storage_arrays(sim_time, dt);
    u_sc_mod_store = memory_allocation_for_u_vector_storage_arrays(sim_time, dt);

    /* Get the aircraft state matrices from text files */
    get_aircraft_state_space_matrices(A_sc, B_sc, A_t, B_t,
                                      aircraft_data_folder_dir);

    return std::make_tuple(sim_time, dt);
}

void control_loop::run(){
    /* Check safety conditions have not been broken */
    // safety_checks.cpp

    /* Store the aircraft data from the timestep */
    store_timestep_data(steps, x_sc, x_t, x_sc_mod, u,
                        u_into_modified_scout, x_sc_store, x_t_store,
                        x_sc_mod_store, u_store, u_sc_mod_store);

    /* Select time to apply step input */
    u[0] = 15*(3.141592654/180);  // M_PI

    /* Run the control loop */
    if(STATE_SPACE_MATRIX_SIZE == 4){
        run_control_loop_full(dt, u, u_into_modified_scout,
                        A_sc, B_sc, x_sc,
                        A_t, B_t, x_t, x_sc_mod,
                        &integral);
    }
    if(STATE_SPACE_MATRIX_SIZE == 2){
        run_control_loop_spo(dt, u, u_into_modified_scout,
                        A_sc, B_sc, x_sc,
                        A_t, B_t, x_t, x_sc_mod,
                        &integral);
    }

    /* Increment the step counter */
    steps += 1;
}

void control_loop::log_data(){
    /* Write the simulation data to file */
    write_sim_data_to_file(steps, dt, x_sc_store, x_t_store, x_sc_mod_store,
                            u_store, u_sc_mod_store);
}

void control_loop::cleanup(){
    /* Free memory used by the storage arrays */
    destroy_arrays(x_sc_store);
    destroy_arrays(x_t_store);
    destroy_arrays(x_sc_mod_store);
    destroy_arrays(u_store);
    destroy_arrays(u_sc_mod_store);
}
