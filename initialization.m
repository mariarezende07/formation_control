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
x_SS_2_0=[[-1;-1;0];[0;0;0]];
x_SS_3_0=[[1;-1;0];[0;0;0]];

%% Control (cooperative reference filter)

lambda_sigma=1;
k_sigma=1;
l_sigma=5;
epsilon_sigma=0.01;

%% Desired trajectory - Circular
x_0_points=[0;0];
x_T_points=[0;0];

y_0_points=[0;1]*10;
y_T_points=[1;1]*10;

theta_0_points=[0;0];
theta_T_points=[0;0];


psi_trajectory=0;
%% Simulation

dt = 0.1; % Step size
final_time = 10;
time_vector = (0:dt:final_time).';
T = final_time/length(x_0_points);

N = length(time_vector);

%% Trajectory

trajectory_calc

%%
state_propagation
%% Ploting
close all

figure

plot(eta_1(1,:),eta_1(2,:));
hold on
plot(eta_2(1,:),eta_2(2,:));
plot(eta_3(1,:),eta_3(2,:));


axis equal
