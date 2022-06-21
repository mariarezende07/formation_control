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
obstacles = [5,0,0];

psi_trajectory=0;
%% Simulation

dt = 0.1; % Step size
final_time = 10;
time_vector = (0:dt:final_time).';

N = length(time_vector);

%% Trajectory

trajectory_calc

%%
lambda_sigma_list=0.01:0.01:0.1;
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
close all

%% Ploting
close all
figure
plot(eta_1(1,:), eta_1(3,:));
hold on
plot(eta_2(1,:), eta_2(3,:));
plot(eta_3(1,:), eta_3(3,:));
plot(eta_4(1,:), eta_4(3,:));
plot(eta_5(1,:), eta_5(3,:));
plot(eta_6(1,:), eta_6(3,:));
plot(eta_7(1,:), eta_7(3,:));

axis equal
%%
figure
plot(x_SS_1(1,:), x_SS_1(3,:));
% hold on
% plot(x_SS_2(1,:), x_SS_2(3,:));
% plot(x_SS_3(1,:), x_SS_3(3,:));
% plot(x_SS_4(1,:), x_SS_4(3,:));
% plot(x_SS_5(1,:), x_SS_5(3,:));
% plot(x_SS_6(1,:), x_SS_6(3,:));
% plot(x_SS_7(1,:), x_SS_7(3,:));

axis equal
