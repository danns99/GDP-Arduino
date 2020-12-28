#include "Control_Model.h"


/*
Runs the simulation
*/
int main(void) {
    int i;
    FILE *f;

    /* Simulation time settings */
    int sim_time = 5;
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

    /* File directories for aircraft data files */
    char scout_data_relative_dir[] = "aircraft_data//";
    char target_data_relative_dir[] = "aircraft_data//";
    char scout_data[] = "scout_state_matrices.txt";
    char target_data[] = "target_state_matrices.txt";

    /* Get aircraft state space matrices */
    read_state_space_matrices_from_file(A_sc, B_sc,
        strcat(scout_data_relative_dir, scout_data));
    read_state_space_matrices_from_file(A_t, B_t,
        strcat(target_data_relative_dir, target_data));

    /* Main time loop */
    while(steps < sim_time*1/dt){
        /* Select time to apply step input */
        if(steps == 99){
            printf("here\n");
            u_old = u[0];
            u[0] = u_old + 0.1;
        }

        /* Solve the state-space equations for the target aircraft and the
        Scout */
        xdot_solve(xdot_t_store, A_t, B_t, x_t, u, dt);
        xdot_solve(xdot_sc_store, A_sc, B_sc, x_sc, u, dt);

        /* Get the control input for the modified Scout */
        /* Get the pitch rate output of the target aircraft */
        q_out_of_target_aircraft = x_t[2];
        /* Get the current error as the pitch rate output of the modified Scout
        aircraft */
        current_error = x_sc_mod[2];
        /* Calculate the error between the pitch rate of the target aircraft and
        the modified Scout */
        u_from_error_sum = error_signal(q_out_of_target_aircraft,
                                        current_error);
        /* Calculate the input to the modified Scout using the PID */
        u_into_modified_scout[0] = PID_controller(u_from_error_sum,
                                                  &error_prior, dt);

        /* Solve the state-space equations for the modified scout */
        xdot_solve(xdot_sc_mod_store, A_sc, B_sc, x_sc_mod,
                       u_into_modified_scout, dt);
        // xdot_solve_mod(xdot_sc_mod_store, A_sc, B_sc, x_sc_mod,
        //                u_into_modified_scout, dt, q_out_of_target_aircraft,
        //                &error_prior);

        /* Store data for the states of the target, Scout and modified Scout
        aircraft at the end of the timestep */
        for (i=0; i<4; i++){
            x_sc_store[steps][i] = x_sc[i];
            x_t_store[steps][i] = x_t[i];
            x_sc_mod_store[steps][i] = x_sc_mod[i];
        }
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
