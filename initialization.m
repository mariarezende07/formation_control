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
x_SS_4_0=[[1;0;0];[0;0;0]];
x_SS_5_0=[[0;1;0];[0;0;0]];

%% Control (cooperative reference filter)

lambda_sigma=0.01;
k_sigma=5;
l_sigma=0.1;
epsilon_sigma=0.1;

%% Desired trajectory

k_trajectory=1;

x_0_points=[0;10;10;0;0]*k_trajectory;
x_T_points=[10;10;0;0;0]*k_trajectory;

y_0_points=[0;0;10;10;0]*k_trajectory;
y_T_points=[0;10;10;0;0]*k_trajectory;

theta_0_points=-[0;0;pi/2;pi;2*pi]*k_trajectory;
theta_T_points=-[0;pi/2;pi;2*pi;2*pi]*k_trajectory;

%% State estimator (KF)

Q_KF=zeros(6,6);
Q_KF(4:5,4:5)=(0.01).^2;
Q_KF(6,6)=(deg2rad(0.1)).^2;

x_SS_0=x_SS_1_0*rand(1)+x_SS_2_0*rand(1)+x_SS_3_0*rand(1)+x_SS_4_0*rand(1)+x_SS_5_0*rand(1);
error=x_SS_0/n_agents;

P_KF=zeros(6,6);
P_KF(4:6,4:6)=diag(abs(mean(error(1:3)*(error(1:3).'))));

Euler_steps=10;

%% Disturbance observer (SMDO)

gamma=1;

gain=2;

k_1=gain*(2*gamma);
k_2=gain*k_1*gamma*((5*k_1+4*gamma)/(2*(k_1-2*gamma)));

K_1=k_1*eye(3);
K_2=k_2*eye(3);
Gamma=2.5*k_2*eye(3);

lambda=0.1*eye(3);

epsilon_saturation=1;

R{n_agents} = 1:n_agents;

for i=1:n_agents
	R{i} =[
		[cos(x_SS_2_0(3)), sin(psi), 0];
		[-sin(theta(psi)), cos(theta(psi)), 0];
		[0,0,1];
	];
end
R{2}=subs(R{2},psi,x_SS_2_0(3));

eta(2,0)=X_SS_2_0(1:3,1);
nu(2,0)=X_SS_2_0(4:6,1);
sigma(2,0)=nu(2,0)+lambda*(R_2.')*eta(2,0);

f_filter=100;

%% Current

v_current_x=0.1;                               % [m/s]
v_current_y=0.1;                               % [m/s]

v_current=sqrt(v_current_x.^2+v_current_y.^2); % [m/s]

noise_power=1;

%% Simulation

step_simulation=1e-3;              

final_time=200;
T=final_time/length(x_0_points);
