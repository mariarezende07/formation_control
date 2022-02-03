%% INITIALIZATION

clearvars
close all
clc

%% MODEL

sliding_variable

%% PARAMETERS

clear A_c b_c nu R M f 

%% Initial position
x_SS_leader_0=[[0;0;0];[0;0;0]];
x_SS_2_0=[[0;0;0];[0;0;0]];
x_SS_3_0=[[0;0;0];[0;0;0]];

%% Control (cooperative reference filter)

lambda_sigma=1;
k_sigma=2;
l_sigma=8;
epsilon_sigma=0.1;

%% Desired trajectory

x_0_points=[0;5;10;]*10;
x_T_points=[5;10;0;]*10;

y_0_points=[0;5;0;]*10;
y_T_points=[5;0;0;]*10;

theta_0_points=[0;0;pi/4;-pi/4]*10;
theta_T_points=[pi/4;-pi/4;pi/2]*10;

psi_trajectory=0;
%% Simulation

dt = 0.1; % Step size
final_time = 100;
time_vector = (0:dt:final_time).';
T = final_time/length(x_0_points);
%% Trajectory

trajectory_calc

%% Ploting
figure
plot(x_SS_leader(1,:),x_SS_leader(2,:));
hold on
plot(x_SS_2(1,:),x_SS_2(2,:));
plot(x_SS_3(1,:),x_SS_3(2,:));
