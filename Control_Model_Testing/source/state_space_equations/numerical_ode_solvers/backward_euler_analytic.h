#ifndef BACKWARD_EULER_ANALYTIC_H
#define BACKWARD_EULER_ANALYTIC_H


double* xdot_solve_backward_euler(
                   double A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                   double B[STATE_SPACE_MATRIX_SIZE],
                   double x[STATE_SPACE_MATRIX_SIZE],
                   double u[STATE_SPACE_MATRIX_SIZE],
                   double dt);

#endif
