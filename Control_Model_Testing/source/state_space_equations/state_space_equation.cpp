#include "state_space_equation.h"


/*
Calculates and returns the state-space equation solution
*/
double state_space_function(int i, double y_option, double A[4][4], double B[4],
                            double x[4], double u[4]){
    double B_times_u;

    B_times_u = B[i]*u[0];
    return A[i][0]*(x[0]+y_option) + A[i][1]*(x[1]+y_option) +
           A[i][2]*(x[2]+y_option) + A[i][3]*(x[3]+y_option) +
           B_times_u;
}
