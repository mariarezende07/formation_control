function [sigma_agent,Q,x_SS_dot]=subsystem_trajectory_1(agent,x_SS_1,x_SS_2,x_SS_3,...
                                                         x_SS_4,x_SS_5,x_SS_6, x_SS_7,...
                                                        lambda_sigma,k_sigma,l_sigma,epsilon_sigma)

%% INPUTS
%% Robot parameters
m = 10;
Iz = 0.1;
l = 0.6; %length of the mobile robot
w = 0.4; %width of the mobile robot
d = w/2;
xbc = 0; ybc= 0; % coordinates of mass center

x_1=x_SS_1(1);
y_1=x_SS_1(2);
theta_1=x_SS_1(3);

u_1=x_SS_1(4);
v_1=x_SS_1(5);
r_1=x_SS_1(6);

u_1_dot=0;
v_1_dot=0;
r_1_dot=0;

x_2=x_SS_2(1);
y_2=x_SS_2(2);
theta_2=x_SS_2(3);

u_2=x_SS_2(4);
v_2=x_SS_2(5);
r_2=x_SS_2(6);

u_2_dot=0;
v_2_dot=0;
r_2_dot=0;

x_3=x_SS_3(1);
y_3=x_SS_3(2);
theta_3=x_SS_3(3);

u_3=x_SS_3(4);
v_3=x_SS_3(5);
r_3=x_SS_3(6);

u_3_dot=0;
v_3_dot=0;
r_3_dot=0;

x_4=x_SS_4(1);
y_4=x_SS_4(2);
theta_4=x_SS_4(3);

u_4=x_SS_4(4);
v_4=x_SS_4(5);
r_4=x_SS_4(6);

u_4_dot=0;
v_4_dot=0;
r_4_dot=0;

x_5=x_SS_5(1);
y_5=x_SS_5(2);
theta_5=x_SS_5(3);

u_5=x_SS_5(4);
v_5=x_SS_5(5);
r_5=x_SS_5(6);

u_5_dot=0;
v_5_dot=0;
r_5_dot=0;

x_6=x_SS_6(1);
y_6=x_SS_6(2);
theta_6=x_SS_6(3);

u_6=x_SS_6(4);
v_6=x_SS_6(5);
r_6=x_SS_6(6);

u_6_dot=0;
v_6_dot=0;
r_6_dot=0;

x_7=x_SS_7(1);
y_7=x_SS_7(2);
theta_7=x_SS_7(3);

u_7=x_SS_7(4);
v_7=x_SS_7(5);
r_7=x_SS_7(6);

u_7_dot=0;
v_7_dot=0;
r_7_dot=0;
%% FORMATION

A_c(:,:,2) =[
 
[ 3*cos(theta_2), 3*sin(theta_2), 0];
[-3*sin(theta_2), 3*cos(theta_2), 0];
[              0,              0, 3];];
 
 
A_c(:,:,3) =[
 
[ 3*cos(theta_3), 3*sin(theta_3), 0];
[-3*sin(theta_3), 3*cos(theta_3), 0];
[              0,              0, 3];];
 
 
A_c(:,:,4) =[
 
[ 3*cos(theta_4), 3*sin(theta_4), 0];
[-3*sin(theta_4), 3*cos(theta_4), 0];
[              0,              0, 3];];
 
 
A_c(:,:,5) =[
 
[ 3*cos(theta_5), 3*sin(theta_5), 0];
[-3*sin(theta_5), 3*cos(theta_5), 0];
[              0,              0, 3];];
 
 
A_c(:,:,6) =[
 
[ 3*cos(theta_6), 3*sin(theta_6), 0];
[-3*sin(theta_6), 3*cos(theta_6), 0];
[              0,              0, 3];];
 
 
A_c(:,:,7) =[
 
[ 3*cos(theta_7), 3*sin(theta_7), 0];
[-3*sin(theta_7), 3*cos(theta_7), 0];
[              0,              0, 3];];

sigma = [[0,       3*lambda_sigma*x_2 - lambda_sigma*x_1 - lambda_sigma*(x_3 - 1) - lambda_sigma*(x_7 + 1) - u_1*cos(theta_1) + 3*u_2*cos(theta_2) - u_3*cos(theta_3) - u_7*cos(theta_7) - v_1*sin(theta_1) + 3*v_2*sin(theta_2) - v_3*sin(theta_3) - v_7*sin(theta_7),       3*lambda_sigma*x_3 - lambda_sigma*x_4 - lambda_sigma*(x_1 + 1) - lambda_sigma*(x_2 + 1) - u_1*cos(theta_1) - u_2*cos(theta_2) + 3*u_3*cos(theta_3) - u_4*cos(theta_4) - v_1*sin(theta_1) - v_2*sin(theta_2) + 3*v_3*sin(theta_3) - v_4*sin(theta_4),       3*lambda_sigma*x_4 - lambda_sigma*x_3 - lambda_sigma*(x_1 + 1) - lambda_sigma*(x_5 + 1) - u_1*cos(theta_1) - u_3*cos(theta_3) + 3*u_4*cos(theta_4) - u_5*cos(theta_5) - v_1*sin(theta_1) - v_3*sin(theta_3) + 3*v_4*sin(theta_4) - v_5*sin(theta_5),       3*lambda_sigma*x_5 - lambda_sigma*x_1 - lambda_sigma*(x_4 - 1) - lambda_sigma*(x_6 + 1) - u_1*cos(theta_1) - u_4*cos(theta_4) + 3*u_5*cos(theta_5) - u_6*cos(theta_6) - v_1*sin(theta_1) - v_4*sin(theta_4) + 3*v_5*sin(theta_5) - v_6*sin(theta_6),       3*lambda_sigma*x_6 - lambda_sigma*x_7 - lambda_sigma*(x_1 - 1) - lambda_sigma*(x_5 - 1) - u_1*cos(theta_1) - u_5*cos(theta_5) + 3*u_6*cos(theta_6) - u_7*cos(theta_7) - v_1*sin(theta_1) - v_5*sin(theta_5) + 3*v_6*sin(theta_6) - v_7*sin(theta_7),       3*lambda_sigma*x_7 - lambda_sigma*x_6 - lambda_sigma*(x_1 - 1) - lambda_sigma*(x_2 - 1) - u_1*cos(theta_1) - u_2*cos(theta_2) - u_6*cos(theta_6) + 3*u_7*cos(theta_7) - v_1*sin(theta_1) - v_2*sin(theta_2) - v_6*sin(theta_6) + 3*v_7*sin(theta_7)];
[0, 3*lambda_sigma*y_2 - lambda_sigma*(y_1 + 2) - lambda_sigma*(y_3 + 1) - lambda_sigma*(y_7 + 1) - v_1*cos(theta_1) + 3*v_2*cos(theta_2) - v_3*cos(theta_3) - v_7*cos(theta_7) + u_1*sin(theta_1) - 3*u_2*sin(theta_2) + u_3*sin(theta_3) + u_7*sin(theta_7), 3*lambda_sigma*y_3 - lambda_sigma*(y_1 + 1) - lambda_sigma*(y_2 - 1) - lambda_sigma*(y_4 + 2) - v_1*cos(theta_1) - v_2*cos(theta_2) + 3*v_3*cos(theta_3) - v_4*cos(theta_4) + u_1*sin(theta_1) + u_2*sin(theta_2) - 3*u_3*sin(theta_3) + u_4*sin(theta_4), 3*lambda_sigma*y_4 - lambda_sigma*(y_1 - 1) - lambda_sigma*(y_3 - 2) - lambda_sigma*(y_5 + 1) - v_1*cos(theta_1) - v_3*cos(theta_3) + 3*v_4*cos(theta_4) - v_5*cos(theta_5) + u_1*sin(theta_1) + u_3*sin(theta_3) - 3*u_4*sin(theta_4) + u_5*sin(theta_5), 3*lambda_sigma*y_5 - lambda_sigma*(y_1 - 2) - lambda_sigma*(y_4 - 1) - lambda_sigma*(y_6 - 1) - v_1*cos(theta_1) - v_4*cos(theta_4) + 3*v_5*cos(theta_5) - v_6*cos(theta_6) + u_1*sin(theta_1) + u_4*sin(theta_4) - 3*u_5*sin(theta_5) + u_6*sin(theta_6), 3*lambda_sigma*y_6 - lambda_sigma*(y_1 - 1) - lambda_sigma*(y_5 + 1) - lambda_sigma*(y_7 - 2) - v_1*cos(theta_1) - v_5*cos(theta_5) + 3*v_6*cos(theta_6) - v_7*cos(theta_7) + u_1*sin(theta_1) + u_5*sin(theta_5) - 3*u_6*sin(theta_6) + u_7*sin(theta_7), 3*lambda_sigma*y_7 - lambda_sigma*(y_1 + 1) - lambda_sigma*(y_2 - 1) - lambda_sigma*(y_6 + 2) - v_1*cos(theta_1) - v_2*cos(theta_2) - v_6*cos(theta_6) + 3*v_7*cos(theta_7) + u_1*sin(theta_1) + u_2*sin(theta_2) + u_6*sin(theta_6) - 3*u_7*sin(theta_7)];
[0,                                                                                                                                     3*r_2 - r_1 - r_3 - r_7 - lambda_sigma*theta_1 + 3*lambda_sigma*theta_2 - lambda_sigma*theta_3 - lambda_sigma*theta_7,                                                                                                                                     3*r_3 - r_2 - r_1 - r_4 - lambda_sigma*theta_1 - lambda_sigma*theta_2 + 3*lambda_sigma*theta_3 - lambda_sigma*theta_4,                                                                                                                                     3*r_4 - r_3 - r_1 - r_5 - lambda_sigma*theta_1 - lambda_sigma*theta_3 + 3*lambda_sigma*theta_4 - lambda_sigma*theta_5,                                                                                                                                     3*r_5 - r_4 - r_1 - r_6 - lambda_sigma*theta_1 - lambda_sigma*theta_4 + 3*lambda_sigma*theta_5 - lambda_sigma*theta_6,                                                                                                                                     3*r_6 - r_5 - r_1 - r_7 - lambda_sigma*theta_1 - lambda_sigma*theta_5 + 3*lambda_sigma*theta_6 - lambda_sigma*theta_7,                                                                                                                                     3*r_7 - r_2 - r_6 - r_1 - lambda_sigma*theta_1 - lambda_sigma*theta_2 - lambda_sigma*theta_6 + 3*lambda_sigma*theta_7];
]; 
b_c = [[0,             k_sigma*tanh((lambda_sigma*x_1 - 3*lambda_sigma*x_2 + lambda_sigma*(x_3 - 1) + lambda_sigma*(x_7 + 1) + u_1*cos(theta_1) - 3*u_2*cos(theta_2) + u_3*cos(theta_3) + u_7*cos(theta_7) + v_1*sin(theta_1) - 3*v_2*sin(theta_2) + v_3*sin(theta_3) + v_7*sin(theta_7))/epsilon_sigma) + lambda_sigma*(u_1*cos(theta_1) + v_1*sin(theta_1)) - 3*lambda_sigma*(u_2*cos(theta_2) + v_2*sin(theta_2)) + lambda_sigma*(u_3*cos(theta_3) + v_3*sin(theta_3)) + lambda_sigma*(u_7*cos(theta_7) + v_7*sin(theta_7)) + l_sigma*(lambda_sigma*x_1 - 3*lambda_sigma*x_2 + lambda_sigma*(x_3 - 1) + lambda_sigma*(x_7 + 1) + u_1*cos(theta_1) - 3*u_2*cos(theta_2) + u_3*cos(theta_3) + u_7*cos(theta_7) + v_1*sin(theta_1) - 3*v_2*sin(theta_2) + v_3*sin(theta_3) + v_7*sin(theta_7)) + u_1_dot*cos(theta_1) + u_3_dot*cos(theta_3) + u_7_dot*cos(theta_7) + v_1_dot*sin(theta_1) + v_3_dot*sin(theta_3) + v_7_dot*sin(theta_7) + r_1*v_1*cos(theta_1) - 3*r_2*v_2*cos(theta_2) + r_3*v_3*cos(theta_3) + r_7*v_7*cos(theta_7) - r_1*u_1*sin(theta_1) + 3*r_2*u_2*sin(theta_2) - r_3*u_3*sin(theta_3) - r_7*u_7*sin(theta_7),             k_sigma*tanh((lambda_sigma*x_4 - 3*lambda_sigma*x_3 + lambda_sigma*(x_1 + 1) + lambda_sigma*(x_2 + 1) + u_1*cos(theta_1) + u_2*cos(theta_2) - 3*u_3*cos(theta_3) + u_4*cos(theta_4) + v_1*sin(theta_1) + v_2*sin(theta_2) - 3*v_3*sin(theta_3) + v_4*sin(theta_4))/epsilon_sigma) + lambda_sigma*(u_1*cos(theta_1) + v_1*sin(theta_1)) + lambda_sigma*(u_2*cos(theta_2) + v_2*sin(theta_2)) - 3*lambda_sigma*(u_3*cos(theta_3) + v_3*sin(theta_3)) + lambda_sigma*(u_4*cos(theta_4) + v_4*sin(theta_4)) + l_sigma*(lambda_sigma*x_4 - 3*lambda_sigma*x_3 + lambda_sigma*(x_1 + 1) + lambda_sigma*(x_2 + 1) + u_1*cos(theta_1) + u_2*cos(theta_2) - 3*u_3*cos(theta_3) + u_4*cos(theta_4) + v_1*sin(theta_1) + v_2*sin(theta_2) - 3*v_3*sin(theta_3) + v_4*sin(theta_4)) + u_1_dot*cos(theta_1) + u_2_dot*cos(theta_2) + u_4_dot*cos(theta_4) + v_1_dot*sin(theta_1) + v_2_dot*sin(theta_2) + v_4_dot*sin(theta_4) + r_1*v_1*cos(theta_1) + r_2*v_2*cos(theta_2) - 3*r_3*v_3*cos(theta_3) + r_4*v_4*cos(theta_4) - r_1*u_1*sin(theta_1) - r_2*u_2*sin(theta_2) + 3*r_3*u_3*sin(theta_3) - r_4*u_4*sin(theta_4),             k_sigma*tanh((lambda_sigma*x_3 - 3*lambda_sigma*x_4 + lambda_sigma*(x_1 + 1) + lambda_sigma*(x_5 + 1) + u_1*cos(theta_1) + u_3*cos(theta_3) - 3*u_4*cos(theta_4) + u_5*cos(theta_5) + v_1*sin(theta_1) + v_3*sin(theta_3) - 3*v_4*sin(theta_4) + v_5*sin(theta_5))/epsilon_sigma) + lambda_sigma*(u_1*cos(theta_1) + v_1*sin(theta_1)) + lambda_sigma*(u_3*cos(theta_3) + v_3*sin(theta_3)) - 3*lambda_sigma*(u_4*cos(theta_4) + v_4*sin(theta_4)) + lambda_sigma*(u_5*cos(theta_5) + v_5*sin(theta_5)) + l_sigma*(lambda_sigma*x_3 - 3*lambda_sigma*x_4 + lambda_sigma*(x_1 + 1) + lambda_sigma*(x_5 + 1) + u_1*cos(theta_1) + u_3*cos(theta_3) - 3*u_4*cos(theta_4) + u_5*cos(theta_5) + v_1*sin(theta_1) + v_3*sin(theta_3) - 3*v_4*sin(theta_4) + v_5*sin(theta_5)) + u_1_dot*cos(theta_1) + u_3_dot*cos(theta_3) + u_5_dot*cos(theta_5) + v_1_dot*sin(theta_1) + v_3_dot*sin(theta_3) + v_5_dot*sin(theta_5) + r_1*v_1*cos(theta_1) + r_3*v_3*cos(theta_3) - 3*r_4*v_4*cos(theta_4) + r_5*v_5*cos(theta_5) - r_1*u_1*sin(theta_1) - r_3*u_3*sin(theta_3) + 3*r_4*u_4*sin(theta_4) - r_5*u_5*sin(theta_5),             k_sigma*tanh((lambda_sigma*x_1 - 3*lambda_sigma*x_5 + lambda_sigma*(x_4 - 1) + lambda_sigma*(x_6 + 1) + u_1*cos(theta_1) + u_4*cos(theta_4) - 3*u_5*cos(theta_5) + u_6*cos(theta_6) + v_1*sin(theta_1) + v_4*sin(theta_4) - 3*v_5*sin(theta_5) + v_6*sin(theta_6))/epsilon_sigma) + lambda_sigma*(u_1*cos(theta_1) + v_1*sin(theta_1)) + lambda_sigma*(u_4*cos(theta_4) + v_4*sin(theta_4)) - 3*lambda_sigma*(u_5*cos(theta_5) + v_5*sin(theta_5)) + lambda_sigma*(u_6*cos(theta_6) + v_6*sin(theta_6)) + l_sigma*(lambda_sigma*x_1 - 3*lambda_sigma*x_5 + lambda_sigma*(x_4 - 1) + lambda_sigma*(x_6 + 1) + u_1*cos(theta_1) + u_4*cos(theta_4) - 3*u_5*cos(theta_5) + u_6*cos(theta_6) + v_1*sin(theta_1) + v_4*sin(theta_4) - 3*v_5*sin(theta_5) + v_6*sin(theta_6)) + u_1_dot*cos(theta_1) + u_4_dot*cos(theta_4) + u_6_dot*cos(theta_6) + v_1_dot*sin(theta_1) + v_4_dot*sin(theta_4) + v_6_dot*sin(theta_6) + r_1*v_1*cos(theta_1) + r_4*v_4*cos(theta_4) - 3*r_5*v_5*cos(theta_5) + r_6*v_6*cos(theta_6) - r_1*u_1*sin(theta_1) - r_4*u_4*sin(theta_4) + 3*r_5*u_5*sin(theta_5) - r_6*u_6*sin(theta_6),             k_sigma*tanh((lambda_sigma*x_7 - 3*lambda_sigma*x_6 + lambda_sigma*(x_1 - 1) + lambda_sigma*(x_5 - 1) + u_1*cos(theta_1) + u_5*cos(theta_5) - 3*u_6*cos(theta_6) + u_7*cos(theta_7) + v_1*sin(theta_1) + v_5*sin(theta_5) - 3*v_6*sin(theta_6) + v_7*sin(theta_7))/epsilon_sigma) + lambda_sigma*(u_1*cos(theta_1) + v_1*sin(theta_1)) + lambda_sigma*(u_5*cos(theta_5) + v_5*sin(theta_5)) - 3*lambda_sigma*(u_6*cos(theta_6) + v_6*sin(theta_6)) + lambda_sigma*(u_7*cos(theta_7) + v_7*sin(theta_7)) + l_sigma*(lambda_sigma*x_7 - 3*lambda_sigma*x_6 + lambda_sigma*(x_1 - 1) + lambda_sigma*(x_5 - 1) + u_1*cos(theta_1) + u_5*cos(theta_5) - 3*u_6*cos(theta_6) + u_7*cos(theta_7) + v_1*sin(theta_1) + v_5*sin(theta_5) - 3*v_6*sin(theta_6) + v_7*sin(theta_7)) + u_1_dot*cos(theta_1) + u_5_dot*cos(theta_5) + u_7_dot*cos(theta_7) + v_1_dot*sin(theta_1) + v_5_dot*sin(theta_5) + v_7_dot*sin(theta_7) + r_1*v_1*cos(theta_1) + r_5*v_5*cos(theta_5) - 3*r_6*v_6*cos(theta_6) + r_7*v_7*cos(theta_7) - r_1*u_1*sin(theta_1) - r_5*u_5*sin(theta_5) + 3*r_6*u_6*sin(theta_6) - r_7*u_7*sin(theta_7),             k_sigma*tanh((lambda_sigma*x_6 - 3*lambda_sigma*x_7 + lambda_sigma*(x_1 - 1) + lambda_sigma*(x_2 - 1) + u_1*cos(theta_1) + u_2*cos(theta_2) + u_6*cos(theta_6) - 3*u_7*cos(theta_7) + v_1*sin(theta_1) + v_2*sin(theta_2) + v_6*sin(theta_6) - 3*v_7*sin(theta_7))/epsilon_sigma) + lambda_sigma*(u_1*cos(theta_1) + v_1*sin(theta_1)) + lambda_sigma*(u_2*cos(theta_2) + v_2*sin(theta_2)) + lambda_sigma*(u_6*cos(theta_6) + v_6*sin(theta_6)) - 3*lambda_sigma*(u_7*cos(theta_7) + v_7*sin(theta_7)) + l_sigma*(lambda_sigma*x_6 - 3*lambda_sigma*x_7 + lambda_sigma*(x_1 - 1) + lambda_sigma*(x_2 - 1) + u_1*cos(theta_1) + u_2*cos(theta_2) + u_6*cos(theta_6) - 3*u_7*cos(theta_7) + v_1*sin(theta_1) + v_2*sin(theta_2) + v_6*sin(theta_6) - 3*v_7*sin(theta_7)) + u_1_dot*cos(theta_1) + u_2_dot*cos(theta_2) + u_6_dot*cos(theta_6) + v_1_dot*sin(theta_1) + v_2_dot*sin(theta_2) + v_6_dot*sin(theta_6) + r_1*v_1*cos(theta_1) + r_2*v_2*cos(theta_2) + r_6*v_6*cos(theta_6) - 3*r_7*v_7*cos(theta_7) - r_1*u_1*sin(theta_1) - r_2*u_2*sin(theta_2) - r_6*u_6*sin(theta_6) + 3*r_7*u_7*sin(theta_7)];
[0, l_sigma*(lambda_sigma*(y_1 + 2) - 3*lambda_sigma*y_2 + lambda_sigma*(y_3 + 1) + lambda_sigma*(y_7 + 1) + v_1*cos(theta_1) - 3*v_2*cos(theta_2) + v_3*cos(theta_3) + v_7*cos(theta_7) - u_1*sin(theta_1) + 3*u_2*sin(theta_2) - u_3*sin(theta_3) - u_7*sin(theta_7)) + lambda_sigma*(v_1*cos(theta_1) - u_1*sin(theta_1)) - 3*lambda_sigma*(v_2*cos(theta_2) - u_2*sin(theta_2)) + lambda_sigma*(v_3*cos(theta_3) - u_3*sin(theta_3)) + lambda_sigma*(v_7*cos(theta_7) - u_7*sin(theta_7)) + k_sigma*tanh((lambda_sigma*(y_1 + 2) - 3*lambda_sigma*y_2 + lambda_sigma*(y_3 + 1) + lambda_sigma*(y_7 + 1) + v_1*cos(theta_1) - 3*v_2*cos(theta_2) + v_3*cos(theta_3) + v_7*cos(theta_7) - u_1*sin(theta_1) + 3*u_2*sin(theta_2) - u_3*sin(theta_3) - u_7*sin(theta_7))/epsilon_sigma) + v_1_dot*cos(theta_1) + v_3_dot*cos(theta_3) + v_7_dot*cos(theta_7) - u_1_dot*sin(theta_1) - u_3_dot*sin(theta_3) - u_7_dot*sin(theta_7) - r_1*u_1*cos(theta_1) + 3*r_2*u_2*cos(theta_2) - r_3*u_3*cos(theta_3) - r_7*u_7*cos(theta_7) - r_1*v_1*sin(theta_1) + 3*r_2*v_2*sin(theta_2) - r_3*v_3*sin(theta_3) - r_7*v_7*sin(theta_7), l_sigma*(lambda_sigma*(y_1 + 1) - 3*lambda_sigma*y_3 + lambda_sigma*(y_2 - 1) + lambda_sigma*(y_4 + 2) + v_1*cos(theta_1) + v_2*cos(theta_2) - 3*v_3*cos(theta_3) + v_4*cos(theta_4) - u_1*sin(theta_1) - u_2*sin(theta_2) + 3*u_3*sin(theta_3) - u_4*sin(theta_4)) + lambda_sigma*(v_1*cos(theta_1) - u_1*sin(theta_1)) + lambda_sigma*(v_2*cos(theta_2) - u_2*sin(theta_2)) - 3*lambda_sigma*(v_3*cos(theta_3) - u_3*sin(theta_3)) + lambda_sigma*(v_4*cos(theta_4) - u_4*sin(theta_4)) + k_sigma*tanh((lambda_sigma*(y_1 + 1) - 3*lambda_sigma*y_3 + lambda_sigma*(y_2 - 1) + lambda_sigma*(y_4 + 2) + v_1*cos(theta_1) + v_2*cos(theta_2) - 3*v_3*cos(theta_3) + v_4*cos(theta_4) - u_1*sin(theta_1) - u_2*sin(theta_2) + 3*u_3*sin(theta_3) - u_4*sin(theta_4))/epsilon_sigma) + v_1_dot*cos(theta_1) + v_2_dot*cos(theta_2) + v_4_dot*cos(theta_4) - u_1_dot*sin(theta_1) - u_2_dot*sin(theta_2) - u_4_dot*sin(theta_4) - r_1*u_1*cos(theta_1) - r_2*u_2*cos(theta_2) + 3*r_3*u_3*cos(theta_3) - r_4*u_4*cos(theta_4) - r_1*v_1*sin(theta_1) - r_2*v_2*sin(theta_2) + 3*r_3*v_3*sin(theta_3) - r_4*v_4*sin(theta_4), l_sigma*(lambda_sigma*(y_1 - 1) - 3*lambda_sigma*y_4 + lambda_sigma*(y_3 - 2) + lambda_sigma*(y_5 + 1) + v_1*cos(theta_1) + v_3*cos(theta_3) - 3*v_4*cos(theta_4) + v_5*cos(theta_5) - u_1*sin(theta_1) - u_3*sin(theta_3) + 3*u_4*sin(theta_4) - u_5*sin(theta_5)) + lambda_sigma*(v_1*cos(theta_1) - u_1*sin(theta_1)) + lambda_sigma*(v_3*cos(theta_3) - u_3*sin(theta_3)) - 3*lambda_sigma*(v_4*cos(theta_4) - u_4*sin(theta_4)) + lambda_sigma*(v_5*cos(theta_5) - u_5*sin(theta_5)) + k_sigma*tanh((lambda_sigma*(y_1 - 1) - 3*lambda_sigma*y_4 + lambda_sigma*(y_3 - 2) + lambda_sigma*(y_5 + 1) + v_1*cos(theta_1) + v_3*cos(theta_3) - 3*v_4*cos(theta_4) + v_5*cos(theta_5) - u_1*sin(theta_1) - u_3*sin(theta_3) + 3*u_4*sin(theta_4) - u_5*sin(theta_5))/epsilon_sigma) + v_1_dot*cos(theta_1) + v_3_dot*cos(theta_3) + v_5_dot*cos(theta_5) - u_1_dot*sin(theta_1) - u_3_dot*sin(theta_3) - u_5_dot*sin(theta_5) - r_1*u_1*cos(theta_1) - r_3*u_3*cos(theta_3) + 3*r_4*u_4*cos(theta_4) - r_5*u_5*cos(theta_5) - r_1*v_1*sin(theta_1) - r_3*v_3*sin(theta_3) + 3*r_4*v_4*sin(theta_4) - r_5*v_5*sin(theta_5), l_sigma*(lambda_sigma*(y_1 - 2) - 3*lambda_sigma*y_5 + lambda_sigma*(y_4 - 1) + lambda_sigma*(y_6 - 1) + v_1*cos(theta_1) + v_4*cos(theta_4) - 3*v_5*cos(theta_5) + v_6*cos(theta_6) - u_1*sin(theta_1) - u_4*sin(theta_4) + 3*u_5*sin(theta_5) - u_6*sin(theta_6)) + lambda_sigma*(v_1*cos(theta_1) - u_1*sin(theta_1)) + lambda_sigma*(v_4*cos(theta_4) - u_4*sin(theta_4)) - 3*lambda_sigma*(v_5*cos(theta_5) - u_5*sin(theta_5)) + lambda_sigma*(v_6*cos(theta_6) - u_6*sin(theta_6)) + k_sigma*tanh((lambda_sigma*(y_1 - 2) - 3*lambda_sigma*y_5 + lambda_sigma*(y_4 - 1) + lambda_sigma*(y_6 - 1) + v_1*cos(theta_1) + v_4*cos(theta_4) - 3*v_5*cos(theta_5) + v_6*cos(theta_6) - u_1*sin(theta_1) - u_4*sin(theta_4) + 3*u_5*sin(theta_5) - u_6*sin(theta_6))/epsilon_sigma) + v_1_dot*cos(theta_1) + v_4_dot*cos(theta_4) + v_6_dot*cos(theta_6) - u_1_dot*sin(theta_1) - u_4_dot*sin(theta_4) - u_6_dot*sin(theta_6) - r_1*u_1*cos(theta_1) - r_4*u_4*cos(theta_4) + 3*r_5*u_5*cos(theta_5) - r_6*u_6*cos(theta_6) - r_1*v_1*sin(theta_1) - r_4*v_4*sin(theta_4) + 3*r_5*v_5*sin(theta_5) - r_6*v_6*sin(theta_6), l_sigma*(lambda_sigma*(y_1 - 1) - 3*lambda_sigma*y_6 + lambda_sigma*(y_5 + 1) + lambda_sigma*(y_7 - 2) + v_1*cos(theta_1) + v_5*cos(theta_5) - 3*v_6*cos(theta_6) + v_7*cos(theta_7) - u_1*sin(theta_1) - u_5*sin(theta_5) + 3*u_6*sin(theta_6) - u_7*sin(theta_7)) + lambda_sigma*(v_1*cos(theta_1) - u_1*sin(theta_1)) + lambda_sigma*(v_5*cos(theta_5) - u_5*sin(theta_5)) - 3*lambda_sigma*(v_6*cos(theta_6) - u_6*sin(theta_6)) + lambda_sigma*(v_7*cos(theta_7) - u_7*sin(theta_7)) + k_sigma*tanh((lambda_sigma*(y_1 - 1) - 3*lambda_sigma*y_6 + lambda_sigma*(y_5 + 1) + lambda_sigma*(y_7 - 2) + v_1*cos(theta_1) + v_5*cos(theta_5) - 3*v_6*cos(theta_6) + v_7*cos(theta_7) - u_1*sin(theta_1) - u_5*sin(theta_5) + 3*u_6*sin(theta_6) - u_7*sin(theta_7))/epsilon_sigma) + v_1_dot*cos(theta_1) + v_5_dot*cos(theta_5) + v_7_dot*cos(theta_7) - u_1_dot*sin(theta_1) - u_5_dot*sin(theta_5) - u_7_dot*sin(theta_7) - r_1*u_1*cos(theta_1) - r_5*u_5*cos(theta_5) + 3*r_6*u_6*cos(theta_6) - r_7*u_7*cos(theta_7) - r_1*v_1*sin(theta_1) - r_5*v_5*sin(theta_5) + 3*r_6*v_6*sin(theta_6) - r_7*v_7*sin(theta_7), l_sigma*(lambda_sigma*(y_1 + 1) - 3*lambda_sigma*y_7 + lambda_sigma*(y_2 - 1) + lambda_sigma*(y_6 + 2) + v_1*cos(theta_1) + v_2*cos(theta_2) + v_6*cos(theta_6) - 3*v_7*cos(theta_7) - u_1*sin(theta_1) - u_2*sin(theta_2) - u_6*sin(theta_6) + 3*u_7*sin(theta_7)) + lambda_sigma*(v_1*cos(theta_1) - u_1*sin(theta_1)) + lambda_sigma*(v_2*cos(theta_2) - u_2*sin(theta_2)) + lambda_sigma*(v_6*cos(theta_6) - u_6*sin(theta_6)) - 3*lambda_sigma*(v_7*cos(theta_7) - u_7*sin(theta_7)) + k_sigma*tanh((lambda_sigma*(y_1 + 1) - 3*lambda_sigma*y_7 + lambda_sigma*(y_2 - 1) + lambda_sigma*(y_6 + 2) + v_1*cos(theta_1) + v_2*cos(theta_2) + v_6*cos(theta_6) - 3*v_7*cos(theta_7) - u_1*sin(theta_1) - u_2*sin(theta_2) - u_6*sin(theta_6) + 3*u_7*sin(theta_7))/epsilon_sigma) + v_1_dot*cos(theta_1) + v_2_dot*cos(theta_2) + v_6_dot*cos(theta_6) - u_1_dot*sin(theta_1) - u_2_dot*sin(theta_2) - u_6_dot*sin(theta_6) - r_1*u_1*cos(theta_1) - r_2*u_2*cos(theta_2) - r_6*u_6*cos(theta_6) + 3*r_7*u_7*cos(theta_7) - r_1*v_1*sin(theta_1) - r_2*v_2*sin(theta_2) - r_6*v_6*sin(theta_6) + 3*r_7*v_7*sin(theta_7)];
[0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         r_1_dot + r_3_dot + r_7_dot + lambda_sigma*r_1 - 3*lambda_sigma*r_2 + lambda_sigma*r_3 + lambda_sigma*r_7 + l_sigma*(r_1 - 3*r_2 + r_3 + r_7 + lambda_sigma*theta_1 - 3*lambda_sigma*theta_2 + lambda_sigma*theta_3 + lambda_sigma*theta_7) + k_sigma*tanh((r_1 - 3*r_2 + r_3 + r_7 + lambda_sigma*theta_1 - 3*lambda_sigma*theta_2 + lambda_sigma*theta_3 + lambda_sigma*theta_7)/epsilon_sigma),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         r_1_dot + r_2_dot + r_4_dot + lambda_sigma*r_1 + lambda_sigma*r_2 - 3*lambda_sigma*r_3 + lambda_sigma*r_4 + l_sigma*(r_1 + r_2 - 3*r_3 + r_4 + lambda_sigma*theta_1 + lambda_sigma*theta_2 - 3*lambda_sigma*theta_3 + lambda_sigma*theta_4) + k_sigma*tanh((r_1 + r_2 - 3*r_3 + r_4 + lambda_sigma*theta_1 + lambda_sigma*theta_2 - 3*lambda_sigma*theta_3 + lambda_sigma*theta_4)/epsilon_sigma),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         r_1_dot + r_3_dot + r_5_dot + lambda_sigma*r_1 + lambda_sigma*r_3 - 3*lambda_sigma*r_4 + lambda_sigma*r_5 + l_sigma*(r_1 + r_3 - 3*r_4 + r_5 + lambda_sigma*theta_1 + lambda_sigma*theta_3 - 3*lambda_sigma*theta_4 + lambda_sigma*theta_5) + k_sigma*tanh((r_1 + r_3 - 3*r_4 + r_5 + lambda_sigma*theta_1 + lambda_sigma*theta_3 - 3*lambda_sigma*theta_4 + lambda_sigma*theta_5)/epsilon_sigma),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         r_1_dot + r_4_dot + r_6_dot + lambda_sigma*r_1 + lambda_sigma*r_4 - 3*lambda_sigma*r_5 + lambda_sigma*r_6 + l_sigma*(r_1 + r_4 - 3*r_5 + r_6 + lambda_sigma*theta_1 + lambda_sigma*theta_4 - 3*lambda_sigma*theta_5 + lambda_sigma*theta_6) + k_sigma*tanh((r_1 + r_4 - 3*r_5 + r_6 + lambda_sigma*theta_1 + lambda_sigma*theta_4 - 3*lambda_sigma*theta_5 + lambda_sigma*theta_6)/epsilon_sigma),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         r_1_dot + r_5_dot + r_7_dot + lambda_sigma*r_1 + lambda_sigma*r_5 - 3*lambda_sigma*r_6 + lambda_sigma*r_7 + l_sigma*(r_1 + r_5 - 3*r_6 + r_7 + lambda_sigma*theta_1 + lambda_sigma*theta_5 - 3*lambda_sigma*theta_6 + lambda_sigma*theta_7) + k_sigma*tanh((r_1 + r_5 - 3*r_6 + r_7 + lambda_sigma*theta_1 + lambda_sigma*theta_5 - 3*lambda_sigma*theta_6 + lambda_sigma*theta_7)/epsilon_sigma),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         r_1_dot + r_2_dot + r_6_dot + lambda_sigma*r_1 + lambda_sigma*r_2 + lambda_sigma*r_6 - 3*lambda_sigma*r_7 + l_sigma*(r_1 + r_2 + r_6 - 3*r_7 + lambda_sigma*theta_1 + lambda_sigma*theta_2 + lambda_sigma*theta_6 - 3*lambda_sigma*theta_7) + k_sigma*tanh((r_1 + r_2 + r_6 - 3*r_7 + lambda_sigma*theta_1 + lambda_sigma*theta_2 + lambda_sigma*theta_6 - 3*lambda_sigma*theta_7)/epsilon_sigma)];
   ];
%% Calculation
sigma_agent = sigma(:,agent);

eval(['[nu,R,M,f]=UGV_dynamics(x_',num2str(agent),',y_',num2str(agent),',theta_',num2str(agent),...
                                ',u_',num2str(agent),',v_',num2str(agent),',r_',num2str(agent),');']);
[x_SS_dot,Q]=cooperative_filter(A_c(:,:,agent),b_c(:,agent),nu,R,M,f);

end
