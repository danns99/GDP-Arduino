#include "sim.h"


/*
 Runs the simulation.
 */
int main(void){
    using namespace std;
    int steps=0;
    double sim_time;
    double dt;

    control_loop control_augmentation_system;
    tie(sim_time, dt) = control_augmentation_system.init();

    while(steps < sim_time*1/dt){
        control_augmentation_system.run();
        steps += 1;
    }

    control_augmentation_system.log_data();
    control_augmentation_system.cleanup();

    return 0;
}
