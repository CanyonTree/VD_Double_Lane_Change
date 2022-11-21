%% Double Lane Change Lab
% MECH 4420

clc; clear all; close all;

g = 9.81;
Steering_Ratio = 14.8;
L = 2.85; % m
b = 1.593;
m = 1756+100+150; % kg
tr = 1.5748;
delta_x = 0.9144;
delta_y = 0.6096;
a = L-b;

t0_velocity = 5;

file1 = 'brooks.bag';
file2 = 'bryce.bag';
file3 = 'nathan.bag';
file4 = 'noah.bag';

data1 = get_data(file1, t0_velocity, Steering_Ratio);
data2 = get_data(file2, t0_velocity, Steering_Ratio);
data3 = get_data(file3, t0_velocity, Steering_Ratio);
data4 = get_data(file4, t0_velocity, Steering_Ratio);
