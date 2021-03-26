#ifndef CONTROL_H
#define CONTROL_H

#include "..//util//memory_management.h"
#include "..//util//file_io.h"
#include "..//control//control_loops//control_loop_spo_model.h"
#include "..//control//control_loops//control_loop_full_longitudinal_model.h"
#include <tuple>


class control_loop{
    public:
        std::tuple<int, double, double> init();
        void run();
        void log_data();
        void cleanup();

        /* Variables for control model settings */
        int steps = 0;
        double sim_time = 0;
        double input_time = 0;
        double dt = 0;
        char aircraft_data_folder_dir[100];

        /* Control vectors */
        double u[STATE_SPACE_MATRIX_SIZE] = {0};
        double u_old=0;

        /* x vectors */
        /* For storing data at the end of each timestep */
        double x_sc[STATE_SPACE_MATRIX_SIZE] = {0};
        double x_t[STATE_SPACE_MATRIX_SIZE] = {0};
        double x_sc_mod[STATE_SPACE_MATRIX_SIZE] = {0};

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

        /* Arrays for storing the aircraft data for all timesteps */
        double* x_sc_store;
        double* x_t_store;
        double* x_sc_mod_store;

        /* Arrays for storing inputs to the aircraft for all timesteps */
        double* u_store;
        double* u_sc_mod_store;
};

#endif
