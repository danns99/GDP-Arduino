#ifndef STATE_SPACE_EQUATION_H
#define STATE_SPACE_EQUATION_H


double state_space_function(int i, double y_option,
                            double A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                            double B[STATE_SPACE_MATRIX_SIZE],
                            double x[STATE_SPACE_MATRIX_SIZE],
                            double u[STATE_SPACE_MATRIX_SIZE]);

#endif
