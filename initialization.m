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
l_sigma=10;
epsilon_sigma=15;

%% Desired trajectory - Circular
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

N = length(time_vector);

%% Trajectory

trajectory_calc

%%
lambda_sigma_list=0.1:0.5:5;
k_sigma_list=1:1:10;
worst_error = 100;
for i=1:10
    lambda_sigma = lambda_sigma_list(i);
    k_sigma = k_sigma_list(i);
    state_propagation
    erro = x_SS_leader(1:3,:) - eta_1;   
    if max(erro,[],'all') < worst_error
        worst_error = max(erro,[],'all');
        k_sigma_best = k_sigma;
        lambda_sigma_best = lambda_sigma;
    end
end
%% 

k_sigma = k_sigma_best;
lambda_sigma = lambda_sigma_best;
state_propagation

%% Ploting
plots
