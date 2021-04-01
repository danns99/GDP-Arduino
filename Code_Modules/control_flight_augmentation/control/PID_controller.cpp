#include "PID_controller.h"


/*
 Calculates and returns the error between the output of the target aircraft and
 the output of the modified Sherwood Scout.
 */
double error_signal(double target_input, double error_from_system_output){
    return (target_input - error_from_system_output);
}


/*
 Calculates and returns the proportional (P) and derivative (D)
 gains using the current error, the previous error and the timestep size. The
 PD gains were determined using MATLAB's response optimizer.
 */
double PD_controller(double error, double *integral, double iteration_time){
    double proportional_term;
    double derivative_term;
    double N = 100.0;
    double K_p = -86.717674561182620; 
    double K_d = 0.637114873357555;

    proportional_term = K_p*error;
    derivative_term = ((K_d * error) - *integral) * N;

    *integral += iteration_time * derivative_term;

    return (proportional_term + derivative_term);
}
