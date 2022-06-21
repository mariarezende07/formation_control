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
x_SS_2_0=[[1;0;0];[0;0;0]];
x_SS_3_0=[[0;-1;0];[0;0;0]];
x_SS_4_0=[[1;-1;0];[0;0;0]];

%% Control (cooperative reference filter)

lambda_sigma=1;
k_sigma=1;
l_sigma=5;
epsilon_sigma=0.01;

%% Desired trajectory - Circular
x_0_points=[0;0];
x_T_points=[0;0];

y_0_points=[0;10];
y_T_points=[10;10];

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

figure

plot(eta_1(1,:),eta_1(2,:));
hold on
plot(eta_2(1,:),eta_2(2,:));
plot(eta_3(1,:),eta_3(2,:));
plot(eta_4(1,:),eta_4(2,:));

axis equal

%% Ploting
close all

figure

plot(x_SS_leader(1,:),x_SS_leader(2,:));
hold on
plot(x_SS_2(1,:),x_SS_2(2,:));
plot(x_SS_3(1,:),x_SS_3(2,:));
plot(x_SS_4(1,:),x_SS_4(2,:));
axis equal
