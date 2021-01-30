#include <stdio.h>
#include <stdlib.h>
#include <string.h>


int read_file(int* sim_time, double* dt, double* input_time, char aircraft_data_folder[100]){
    int i;
    FILE *f;
    int test;
    
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
            fscanf(f, "%*s %*s %*s %*s %*s %*s %*s %*s %*s %s", aircraft_data_folder);
        }
    }

    if (fclose(f) != 0 ) {
        printf("File could not be closed.\n");
        return -1;
    }

    return 0;
}


int print_test(int sim_time, double dt, double input_time, char aircraft_data_folder[100]){
    printf("%d\n", sim_time);
    printf("%f\n", dt);
    printf("%f\n", input_time);
    printf("%s\n", aircraft_data_folder);

    char scout_data[] = "scout_state_matrices.txt";

    strcat(aircraft_data_folder, scout_data);
    printf("%s\n", aircraft_data_folder);

    return 0;
}


int main(void){
    int sim_time=0;
    double dt=0;
    double input_time=0;
    char aircraft_data_folder[100];

    read_file(&sim_time, &dt, &input_time, aircraft_data_folder);

    print_test(sim_time, dt, input_time, aircraft_data_folder);

    return 0;
}
