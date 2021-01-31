#include "state_space_equation.h"


/*
 Calculates and returns the state-space equation solution.
 */
double state_space_function(int i, double y_option,
                            double A[STATE_SPACE_MATRIX_SIZE][STATE_SPACE_MATRIX_SIZE],
                            double B[STATE_SPACE_MATRIX_SIZE],
                            double x[STATE_SPACE_MATRIX_SIZE],
                            double u[STATE_SPACE_MATRIX_SIZE]){
    double B_times_u;

    B_times_u = B[i]*u[0];
    if(STATE_SPACE_MATRIX_SIZE == 4){
       return A[i][0]*(x[0]+y_option) + A[i][1]*(x[1]+y_option) +
              A[i][2]*(x[2]+y_option) + A[i][3]*(x[3]+y_option) +
              B_times_u;
    }
    else if(STATE_SPACE_MATRIX_SIZE == 2){
       return A[i][0]*(x[0]+y_option) + A[i][1]*(x[1]+y_option) + B_times_u;       
    }
}
