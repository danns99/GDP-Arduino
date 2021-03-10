%% clear workspace and console
clear all
clc

%% simulation settings
stop_time = 10;  % seconds

%% motor values
R_armature = 3.9;  % Resistance (Ohm)
motor_inductance = 12e-6;  % Inductance (H)
motor_constant = 0.072e-3*60/(2*pi);  % K (V/(rad/s))
motor_inertia = 0.01;  % J (g*cm^2)
motor_damping_constant = 3.5077e-6;  % B (N*m/(rad/s))

%% motor state-space
dc_motor_state_space_A = [0 1 0;
                          0 -motor_damping_constant/motor_inertia motor_constant/motor_inertia; %0.0274
                          0 -motor_constant/motor_inductance -R_armature/motor_inductance];
dc_motor_state_space_B = [0;
                          0;
                          1/motor_inductance];
dc_motor_state_space_C = [0 0 1];
dc_motor_state_space_D = 0;

%% run simulink model
sim('force_feedback_model.slx')
