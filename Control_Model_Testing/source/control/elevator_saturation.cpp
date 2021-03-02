# include "elevator_saturation.h"


/*
 Checks if the current elevator angle from the control model exceeds the
 maximum or minimum elevator angle. If the current elevator angle exceeds the
 limits then the elevator angle is set to the max/min value within the allowable
 range of elevator angles. 
 */
double saturate_elevator(double current_elevator_angle){
    double saturated_elevator_angle;
    double max_elevator_up_angle = 25*3/180;
    double min_elevator_up_angle = -25*3/180;

    if(current_elevator_angle > max_elevator_up_angle){
        saturated_elevator_angle = max_elevator_up_angle;
    }
    if(current_elevator_angle < min_elevator_up_angle){
        saturated_elevator_angle = min_elevator_up_angle;
    }
    else{
        saturated_elevator_angle = current_elevator_angle;
    }

    return saturated_elevator_angle;
}
