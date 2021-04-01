#include <stdio.h>


double x_dot_func(double x){
    return -9*x;
}


int main(void){    
    double N=0;
    double x_fixed;
    double x_dot;
    double x=1.3;
    double dt=0.1;

    /* Use Euler's method to calculate start point */
    x_fixed = x;
    x_dot = x_dot_func(x);
    x = x + x_dot*dt; 
    while(N<10){
        x_dot = x_dot_func(x);
        x = x_fixed + x_dot*dt;
        N++;
        printf("x is: %f\n", x);
    }

    return 0;
}