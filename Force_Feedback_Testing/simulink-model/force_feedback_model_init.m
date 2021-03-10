%% clear workspace and console
clear all
clc

%% simulation settings
stop_time = 10;  % seconds

%% motor values
R_armature = 3;  % Resistance (Ohm)
motor_inductance = 0.5;  % Inductance (H)
motor_constant = 0.01;  % K (V/(rad/s))
motor_inertia = 0.01;  % J (kg*m^2)
motor_damping_constant = 0.1;  % B (N*m/(rad/s))

%% motor state-space
dc_motor_state_space_A = [0 1 0;
                          0 -motor_damping_constant/motor_inertia motor_constant/motor_inertia;
                          0 -motor_constant/motor_inductance -R_armature/motor_inductance];
dc_motor_state_space_B = [0;
                          0;
                          1/motor_inductance];
dc_motor_state_space_C = [0 0 1];
dc_motor_state_space_D = 0;

%% run simulink model
sim('force_feedback_model.slx')
