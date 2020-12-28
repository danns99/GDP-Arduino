#ifndef STATE_SPACE_EQUATION_H
#define STATE_SPACE_EQUATION_H


double state_space_function(int i, double y_option, double A[4][4], double B[4],
                            double x[4], double u[4]);

#endif
