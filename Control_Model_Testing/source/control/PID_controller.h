#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H


double error_signal(double target_input, double error_from_system_output);

double PD_controller(double error, double *integral, double iteration_time);

#endif
