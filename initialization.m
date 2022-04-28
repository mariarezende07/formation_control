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

lambda_sigma=0.1;
k_sigma=1;
l_sigma=2;
epsilon_sigma=0.1;

%% Desired trajectory - Field points

target_point = [20,0,0];
obstacles = [5,0,0];

psi_trajectory=0;
%% Simulation

dt = 0.1; % Step size
final_time = 100;
time_vector = (0:dt:final_time).';
%% Trajectory

trajectory_calc


%% Ploting
close all

figure
plot(x_SS_leader(1,:),x_SS_leader(2,:));
hold on
plot(x_SS_leader(1,:),x_SS_leader(3,:));
% plot(x_SS_2(1,:),x_SS_2(2,:));
% plot(x_SS_3(1,:),x_SS_3(2,:));
% plot(x_SS_4(1,:),x_SS_4(2,:));
% plot(x_SS_5(1,:),x_SS_5(2,:));
% plot(x_SS_6(1,:),x_SS_6(2,:));
% plot(x_SS_7(1,:),x_SS_7(2,:));

plot(5, 0, 'bo', 'MarkerSize', 7);


% for i=1:100:length(time_vector)
% 
%     
%     plot([x_SS_2(1,i),x_SS_3(1,i)],[x_SS_2(2,i),x_SS_3(2,i)],'color','k','LineWidth',1);
%     plot([x_SS_2(1,i),x_SS_7(1,i)],[x_SS_2(2,i),x_SS_7(2,i)],'color','k','LineWidth',1);
%     
%     plot([x_SS_5(1,i),x_SS_4(1,i)],[x_SS_5(2,i),x_SS_4(2,i)],'color','k','LineWidth',1);
%     plot([x_SS_5(1,i),x_SS_6(1,i)],[x_SS_5(2,i),x_SS_6(2,i)],'color','k','LineWidth',1);
%     
%     plot([x_SS_6(1,i),x_SS_7(1,i)],[x_SS_6(2,i),x_SS_7(2,i)],'color','k','LineWidth',1);
%     
%     plot([x_SS_3(1,i),x_SS_4(1,i)],[x_SS_3(2,i),x_SS_4(2,i)],'color','k','LineWidth',1);
% end

axis equal