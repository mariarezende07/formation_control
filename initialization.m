%% INITIALIZATION

clearvars
close all
clc

%% MODEL

sliding_variable

%% PARAMETERS

clear A_c nu R M f 

%% Initial position

x_SS_1_0=[[0;0;0];[0;0;0]];
x_SS_2_0=[[-1;0;0];[0;0;0]];
x_SS_3_0=[[0;-1;0];[0;0;0]];

%% Control (cooperative reference filter)

lambda_sigma=0.1;
k_sigma=5;
l_sigma=10;
epsilon_sigma=0.1;
m = 10;

%% Desired trajectory

x_0_points=[0;10;10];
x_T_points=[10;10;0;];

y_0_points=[0;0;10;];
y_T_points=[0;10;10;];

theta_0_points=-[0;0;pi/2;];
theta_T_points=-[0;pi/2;pi;];

psi_trajectory=0;

%% Simulation

dt = 0.1; % Step size
final_time = 100;
time_vector = (0:dt:final_time).';
T = final_time/length(x_0_points);
%% Trajectory

trajectory_calc

%% Plot
% figure
% plot(time_vector,sigma_1(1,:));
% figure
% plot(time_vector,sigma_1(2,:));
% figure
% plot(time_vector,sigma_1(3,:));
% 
% figure
% plot(x_SS_1(1,:),x_SS_1(2,:));
% 
% hold on;
% plot(x_SS_2(1,:),x_SS_2(2,:));
% plot(x_SS_3(1,:),x_SS_3(2,:));
