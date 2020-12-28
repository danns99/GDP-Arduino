#include "file_io.h"


/* 
Reads in the aircraft state space matrix from a text file. Then writes the
data to arrays A and B which are passed to the function.
*/
int read_state_space_matrices_from_file(double A[4][4], double B[4],
                                      char* file_dir){
    int i;
    FILE *f;

    if ((f=fopen(file_dir,"r"))==NULL) {
        printf("Cannot open file for reading.\n");
        return -1;
    }
    for (i=0; i<8; i++) {
        if (feof(f))
            break;
        if (i<4){
            fscanf(f, "%lf %lf %lf %lf", &(A[i][0]), &(A[i][1]),
                                         &(A[i][2]), &(A[i][3]));
        }
        if (i>6){
            fscanf(f, "%lf %lf %lf %lf", &(B[0]), &(B[1]),
                                         &(B[2]), &(B[3]));
        }
    }
    
    if (fclose(f) != 0 ) {
        printf("File could not be closed.\n");
        return -1;
    }
    return 0;
}


/*
Gets the aircraft state space matrices for the Scout and the target aircraft.
*/
int get_aircraft_state_space_matrices(double A_sc[4][4], double *B_sc,
                                      double A_t[4][4], double *B_t){
    /* File directories for aircraft data files */
    char scout_data_relative_dir[] = "..//aircraft_data//";
    char target_data_relative_dir[] = "..//aircraft_data//";
    char scout_data[] = "scout_state_matrices.txt";
    char target_data[] = "target_state_matrices.txt";

    /* Get aircraft state space matrices */
    read_state_space_matrices_from_file(A_sc, B_sc,
        strcat(scout_data_relative_dir, scout_data));
    read_state_space_matrices_from_file(A_t, B_t,
        strcat(target_data_relative_dir, target_data));

    return 0;
}


/*
Writes the simulation data to a text file at the end of the simulation.
*/
int write_sim_data_to_file(int steps, double dt, double **x_sc_store,
                           double **x_t_store, double **x_sc_mod_store){
    int i;
    FILE *f;

    f=fopen("test_data.txt","w");
    for(i=0; i<steps; i++){
        fprintf(f, "%f %f %f %f %f %f %f %f %f %f %f %f %f \n",
                i*dt, x_sc_store[i][0], x_sc_store[i][1], x_sc_store[i][2],
                x_sc_store[i][3], x_t_store[i][0], x_t_store[i][1],
                x_t_store[i][2], x_t_store[i][3], x_sc_mod_store[i][0],
                x_sc_mod_store[i][1], x_sc_mod_store[i][2],
                x_sc_mod_store[i][3]);
    }
    return 0; 
}


/*
Stores data for the states of the target, Scout and modified Scout
aircraft at the end of the timestep
*/
int store_timestep_data(int steps, double *x_sc, double *x_t, double *x_sc_mod,
                        double **x_sc_store, double **x_t_store,
                        double **x_sc_mod_store){
    int i;

    for (i=0; i<4; i++){
        x_sc_store[steps][i] = x_sc[i];
        x_t_store[steps][i] = x_t[i];
        x_sc_mod_store[steps][i] = x_sc_mod[i];
    }
    return 0;
}
