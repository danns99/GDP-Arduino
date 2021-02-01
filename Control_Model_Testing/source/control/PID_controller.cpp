#include "PID_controller.h"


/*
 Calculates and returns the error between the output of the target aircraft and
 the output of the modified Sherwood Scout.
 */
double error_signal(double target_input, double error_from_system_output){
    return (target_input - error_from_system_output);
}


/*
 Calculates and returns the proportional (P) integral (I) and derivative (D)
 gains using the current error, the previous error and the timestep size. The
 PID gains were determined using MATLAB's response optimizer.
 */
double PID_controller(double error, double *error_prior, double iteration_time){
    double proportional_term;
    double integral_term=0;
    double derivative_term;
    double K_p = -98.687493079853600;
    double K_i = -19.396423994327110;
    double K_d = -0.018045698057602;

    proportional_term = K_p*error;
    integral_term += K_i*(error*iteration_time);
    derivative_term = K_d*((error - (*error_prior)) / iteration_time);

    *error_prior = error;
    return (proportional_term + integral_term + derivative_term);
}
