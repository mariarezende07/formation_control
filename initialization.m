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
x_SS_2_0=[[0;2;0];[0;0;0]];
x_SS_3_0=[[1;1;0];[0;0;0]];
x_SS_4_0=[[1;-1;0];[0;0;0]];
x_SS_5_0=[[0;-2;0];[0;0;0]];
x_SS_6_0=[[-1;-1;0];[0;0;0]];
x_SS_7_0=[[-1;1;0];[0;0;0]];

%% Control (cooperative reference filter)

lambda_sigma=1;
k_sigma=1;
l_sigma=5;
epsilon_sigma=0.01;

%% Desired trajectory - Field points
target_point = [20,0,0];
obstacles = [5,0,0]./target_point;

psi_trajectory=0;
%% Simulation

dt = 0.1; % Step size
final_time = 10;
time_vector = (0:dt:final_time).';

N = length(time_vector);

%% Trajectory

trajectory_calc

% %%
% lambda_sigma_list=0.01:0.01:0.1;
% k_sigma_list=1:1:10;
% worst_error = 100;
% 
% for i=1:10
%     lambda_sigma = lambda_sigma_list(i);
%     k_sigma = k_sigma_list(i);
%     state_propagation
%     erro = x_SS_leader(1:3,:) - eta_1;   
%     if max(erro,[],'all') < worst_error
%         worst_error = max(erro,[],'all');
%         k_sigma_best = k_sigma;
%         lambda_sigma_best = lambda_sigma;
%     end
% end
%% 
k_sigma_best = 10;
lambda_sigma_best = 0.1;
k_sigma = k_sigma_best; % 10
lambda_sigma = lambda_sigma_best; % 0.1
state_propagation
%% Ploting

