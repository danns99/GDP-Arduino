#include "Control_Model.h"


/*
Runs the simulation
*/
int main(void) {
    int i;
    FILE *f;

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
    double q_out_of_target_aircraft;
    double u_from_error_sum;
    double u_into_modified_scout[4] = {0};
    double current_error = 0;
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

    /* Main time loop */
    while(steps < sim_time*1/dt){
        /* Select time to apply step input */
        float difference = (float) steps - input_time*1/dt;
        float tol_difference = 0.001;
        if((-tol_difference <= difference) && (difference <= tol_difference)){
            printf("here\n");
            u_old = u[0];
            u[0] = u_old + 0.1;
        }

        /* Run the control loop */
        run_control_loop(dt, u, u_into_modified_scout,
                         A_sc, B_sc, x_sc,
                         A_t, B_t, x_t, x_sc_mod, xdot_t_store,
                         xdot_sc_store, xdot_sc_mod_store,
                         error_prior);

        /* Store the aircraft data from the timestep */
        store_timestep_data(steps, x_sc, x_t, x_sc_mod, x_sc_store, x_t_store,
                            x_sc_mod_store);

        /* Increment the step counter */
        steps += 1;
    }

    printf("%f\n", error_prior);

    /* Write the simulation data to file */
    write_sim_data_to_file(steps, dt, x_sc_store, x_t_store, x_sc_mod_store);

    /* Free memory used by the storage arrays */
    destroy_arrays(x_sc_store);
    destroy_arrays(x_t_store);
    destroy_arrays(x_sc_mod_store);

    return 0;
}
