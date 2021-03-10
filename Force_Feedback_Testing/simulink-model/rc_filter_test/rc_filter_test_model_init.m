%% clear workspace and console
clear all
clc

%% simulation settings
stop_time = 10;  % seconds

%% set value of load resistor
R_load = 1000;  % Resistance (Ohm)

%% set value of voltage sense resistor
R_sense = 1;  % Resistance (Ohm)

%% set values of RC filter
R = 500;  % Resistance (Ohm)
C = 500e-6;  % Capacitance (F)

%% calculate current: I = V/R
current_gain = 1/R_sense;

%% run simulink model
sim('rc_filter_test.slx')
