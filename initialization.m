%% INITIALIZATION

clearvars
close all
clc

%% MODEL

sliding_variable

%% PARAMETERS

clear A_c b_c nu R M f 

%% Initial position

x_SS_1_0=[[0;0;0];[0;0;0]];
x_SS_2_0=[[-1;0;0];[0;0;0]];
x_SS_3_0=[[0;-1;0];[0;0;0]];

%% INPUTS
x_1=x_SS_1_0(1);
y_1=x_SS_1_0(2);
theta_1=x_SS_1_0(3);

u_1=x_SS_1_0(4);
v_1=x_SS_1_0(5);
r_1=x_SS_1_0(6);

u_1_dot=0;
v_1_dot=0;
r_1_dot=0;

x_2=x_SS_2_0(1);
y_2=x_SS_2_0(2);
theta_2=x_SS_2_0(3);

u_2=x_SS_2_0(4);
v_2=x_SS_2_0(5);
r_2=x_SS_2_0(6);

u_2_dot=0;
v_2_dot=0;
r_2_dot=0;

x_3=x_SS_3_0(1);
y_3=x_SS_3_0(2);
theta_3=x_SS_3_0(3);

u_3=x_SS_3_0(4);
v_3=x_SS_3_0(5);
r_3=x_SS_3_0(6);

u_3_dot=0;
v_3_dot=0;
r_3_dot=0;


%% Control (cooperative reference filter)

lambda_sigma=0.1;
k_sigma=5;
l_sigma=10;
epsilon_sigma=0.1;
m = 10;


%% Desired trajectory-1

x_0_points=[0;10;10];
x_T_points=[10;10;0;];

y_0_points=[0;0;10;];
y_T_points=[0;10;10;];

theta_0_points=-[0;0;pi/2;];
theta_T_points=-[0;pi/2;pi;];


%% State estimator (KF)

Q_KF=zeros(6,6);
Q_KF(4:5,4:5)=(0.01).^2;
Q_KF(6,6)=(deg2rad(0.1)).^2;

x_SS_0=x_SS_1_0*rand(1)+x_SS_2_0*rand(1)+x_SS_3_0*rand(1);
error=x_SS_0/n_agents;

P_KF=zeros(6,6);
P_KF(4:6,4:6)=diag(abs(mean(error(1:3)*(error(1:3).'))));

Euler_steps=10;

% %% Sensor

% R_USBL=((0.02).^2)*eye(2);
% f_USBL=5;

% r_theta_IMU=(deg2rad(0.4)).^2;
% f_IMU=10;

% R_DVL=((1e-3).^2)*eye(2);
% f_DVL=5;

% r_r_IMU=(deg2rad(0.2)).^2;

% %% Disturbance observer (SMDO)

% gamma=1;

% gain=2;

% k_1=gain*(2*gamma);
% k_2=gain*k_1*gamma*((5*k_1+4*gamma)/(2*(k_1-2*gamma)));

% K_1=k_1*eye(3);
% K_2=k_2*eye(3);
% Gamma=2.5*k_2*eye(3);

% lambda=0.1*eye(3);

% ethetalon_saturation=1;

% R=[[cos(theta),-sin(theta),0];[sin(theta),cos(theta),0];[0,0,1]];
% R_2=eval(subs(R,theta,x_SS_2_0_0(3)));
% R_3=eval(subs(R,theta,x_SS_3_0(3)));
% R_4=eval(subs(R,theta,x_SS_4_0(3)));
% R_5=eval(subs(R,theta,x_SS_5_0(3)));

% eta_2_0=x_SS_2_0_0(1:3,1);
% nu_2_0=x_SS_2_0_0(4:6,1);
% sigma_2_0=nu_2_0+lambda*(R_2.')*eta_2_0;

% eta_3_0=x_SS_3_0(1:3,1);
% nu_3_0=x_SS_3_0(4:6,1);
% sigma_3_0=nu_3_0+lambda*(R_3.')*eta_3_0;

% f_filter=100;

% %% Current

current_status=0;

v_current_x=0.1;                               % [m/s]
v_current_y=0.1;                               % [m/s]

v_current=sqrt(v_current_x.^2+v_current_y.^2); % [m/s]

noise_power=1;

%% Simulation

step_simulation=1e-3;              

final_time=300;
T=final_time/length(x_0_points);

[sigma_1,Q_1,x_SS_1_dot] = subsystem_trajectory_1(x_SS_1_0,x_SS_2_0,x_SS_3_0,...
    lambda_sigma,k_sigma,l_sigma,epsilon_sigma);
