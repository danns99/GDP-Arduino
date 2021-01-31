#include "file_io.h"


/*
Reads in the simulation settings from sim_settings.txt. The text file should be
placed in the same folder as the executable.
*/
int read_sim_settings(int* sim_time, double* dt, double* input_time,
                      char aircraft_data_folder_dir[200]){
    int i;
    FILE *f;

    if((f=fopen("sim_settings.txt","r"))==NULL){
        printf("Cannot open file for reading.\n");
        return -1;
    }

    for(i=0; i<4; i++){
        if(i==0){
            fscanf(f, "%*s %*s %*s %d", sim_time);
        }
        if(i==1){
            fscanf(f, "%*s %*s %*s %lf", dt);
        }
        if(i==2){
            fscanf(f, "%*s %*s %*s %*s %*s %*s %lf", input_time);
        }
        if(i==3){
            fscanf(f, "%*s %*s %*s %*s %*s %*s %*s %*s %*s %s",
                   aircraft_data_folder_dir);
        }
    }

    if (fclose(f) != 0 ) {
        printf("File could not be closed.\n");
        return -1;
    }

    return 0;
}


/* 
Reads in the aircraft state space matrix from a text file. Then writes the
data to arrays A and B which are passed to the function.
*/
int read_state_space_matrices_from_file(double A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                                        double B[STATE_SPACE_MATRIX_SIZE],
                                        char* file_dir){
    int i;
    FILE *f;

    if ((f=fopen(file_dir,"r"))==NULL) {
        printf("Cannot open file for reading.\n");
        return -1;
    }

    if (STATE_SPACE_MATRIX_SIZE == 4){
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
    }
    if (STATE_SPACE_MATRIX_SIZE == 2){
        for (i=0; i<8; i++) {
            if (feof(f))
                break;
            if(i==0 || i==3){
                fscanf(f, "%*f %*f %*f %*f");
            }
            if (i==1){
                fscanf(f, "%*f %lf %lf %*f", &(A[0][0]), &(A[0][1]));
            }
            if (i==2){
                fscanf(f, "%*f %lf %lf %*f", &(A[1][0]), &(A[1][1]));
            }
            if (i>6){
                fscanf(f, "%*f %lf %lf %*f", &(B[0]), &(B[1]));
            }
        }
    }
    else{
        printf("Warning invalid size of aircraft state space matrices.\n");
        return -1;
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
int get_aircraft_state_space_matrices(double A_sc[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                                      double B_sc[STATE_SPACE_MATRIX_SIZE],
                                      double A_t[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                                      double B_t[STATE_SPACE_MATRIX_SIZE],
                                      char aircraft_data_folder_dir[200]){
    /* File directories for aircraft data files */
    char data_dir_scout[200];
    char data_dir_target[200];
    /* File names for aircraft data files */
    char scout_file[] = "scout_state_matrices.txt";
    char target_file[] = "target_state_matrices.txt";

    /* Copy the aircraft data folder directory */
    strcpy(data_dir_scout, aircraft_data_folder_dir);
    strcpy(data_dir_target, aircraft_data_folder_dir);

    /* Create the directory to each aircraft data file by concatenating the
    file directory to the aircraft data files and the name of the aircraft data
    file */
    strcat(data_dir_scout, scout_file);
    strcat(data_dir_target, target_file);

    /* Get the aircraft state space matrices */
    read_state_space_matrices_from_file(A_sc, B_sc, data_dir_scout);
    read_state_space_matrices_from_file(A_t, B_t, data_dir_target);

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

    if(STATE_SPACE_MATRIX_SIZE == 4){
        for(i=0; i<steps; i++){
            fprintf(f, "%f %f %f %f %f %f %f %f %f %f %f %f %f \n",
                    i*(dt), x_sc_store[i][0], x_sc_store[i][1], x_sc_store[i][2],
                    x_sc_store[i][3], x_t_store[i][0], x_t_store[i][1],
                    x_t_store[i][2], x_t_store[i][3], x_sc_mod_store[i][0],
                    x_sc_mod_store[i][1], x_sc_mod_store[i][2],
                    x_sc_mod_store[i][3]);
        }
    }
    if(STATE_SPACE_MATRIX_SIZE == 2){
        for(i=0; i<steps; i++){
            fprintf(f, "%f %f %f %f %f %f %f \n",
                    i*(dt), x_sc_store[i][0], x_sc_store[i][1], x_t_store[i][0],
                    x_t_store[i][1], x_sc_mod_store[i][0],
                    x_sc_mod_store[i][1]);
        }
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

    for (i=0; i<STATE_SPACE_MATRIX_SIZE; i++){
        x_sc_store[steps][i] = x_sc[i];
        x_t_store[steps][i] = x_t[i];
        x_sc_mod_store[steps][i] = x_sc_mod[i];
    }
    return 0;
}
