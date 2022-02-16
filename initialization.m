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
x_SS_2_0=[[0;1;0];[0;0;0]];
x_SS_3_0=[[1;0;0];[0;0;0]];
x_SS_4_0=[[1;-1;0];[0;0;0]];
x_SS_5_0=[[-1;-1;0];[0;0;0]];
x_SS_6_0=[[-1;0;0];[0;0;0]];

%% Control (cooperative reference filter)

lambda_sigma=0.1;
k_sigma=1;
l_sigma=2;
epsilon_sigma=0.1;

%% Desired trajectory - Circular
% x_0_points=0;
% 
% y_0_points=0;
% 
% theta_0_points=0;

x_0_points=[0;5;10;];
x_T_points=[5;10;0;];

y_0_points=[0;5;0;];
y_T_points=[5;0;0;];

theta_0_points=[0;0;pi/4;-pi/4];
theta_T_points=[pi/4;-pi/4;pi/2];

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
plot(x_SS_4(1,:),x_SS_4(2,:));
plot(x_SS_5(1,:),x_SS_5(2,:));
plot(x_SS_6(1,:),x_SS_6(2,:));
plot(2, 3, '.r', 'MarkerSize', 50);


% 
% 
% figure
% plot(time_vector, sigma_2(1,:));
% hold on
% plot(time_vector, sigma_2(2,:));
% plot(time_vector, sigma_2(3,:));
% 
% figure
% plot(time_vector, Q_2(1,:));
% hold on
% plot(time_vector, Q_2(2,:));
% plot(time_vector, Q_2(3,:));
% 
% figure
% subplot(3,1,1)
% plot(time_vector, x_SS_leader(1,:));
% hold on
% plot(time_vector, x_SS_leader(2,:));
% plot(time_vector, x_SS_leader(3,:));
% xlabel('Tempo (s)')
% ylabel('Deslocamento (m, rad)')
% legend('x_{l}','y_{l}','\psi_{l}','location','best')
% 
% subplot(3,1,2)
% plot(time_vector, x_SS_2(1,:));
% hold on
% plot(time_vector, x_SS_2(2,:));
% plot(time_vector, x_SS_2(3,:));
% 
% subplot(3,1,3)
% plot(time_vector, x_SS_3(1,:));
% hold on
% plot(time_vector, x_SS_3(2,:));
% plot(time_vector, x_SS_3(3,:));
% 
