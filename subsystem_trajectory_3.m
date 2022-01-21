function [sigma_3,Q_3,x_SS_3_dot]=subsystem_trajectory_3(x_SS_1,x_SS_2,x_SS_3,...
    lambda_sigma,k_sigma,l_sigma,epsilon_sigma)

%% INPUTS

x_1=x_SS_1(1);
y_1=x_SS_1(2);
theta_1=x_SS_1(3);

u_1=x_SS_1(4);
v_1=x_SS_1(5);
r_1=x_SS_1(6);

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

%% FORMATION

A_c=[[ 2*cos(theta_3), 2*sin(theta_3), 0];
     [-2*sin(theta_3), 2*cos(theta_3), 0];
     [              0,              0, 2];];

b_c=[k_sigma*tanh((lambda_sigma*x_2 - 2*lambda_sigma*x_3 + lambda_sigma*(x_1 + 1) + u_1*cos(theta_1) + u_2*cos(theta_2) - 2*u_3*cos(theta_3) + v_1*sin(theta_1) + v_2*sin(theta_2) - 2*v_3*sin(theta_3))/epsilon_sigma) + lambda_sigma*(u_1*cos(theta_1) + v_1*sin(theta_1)) + lambda_sigma*(u_2*cos(theta_2) + v_2*sin(theta_2)) - 2*lambda_sigma*(u_3*cos(theta_3) + v_3*sin(theta_3)) + l_sigma*(lambda_sigma*x_2 - 2*lambda_sigma*x_3 + lambda_sigma*(x_1 + 1) + u_1*cos(theta_1) + u_2*cos(theta_2) - 2*u_3*cos(theta_3) + v_1*sin(theta_1) + v_2*sin(theta_2) - 2*v_3*sin(theta_3)) + u_1_dot*cos(theta_1) + u_2_dot*cos(theta_2) + v_1_dot*sin(theta_1) + v_2_dot*sin(theta_2) + r_1*v_1*cos(theta_1) + r_2*v_2*cos(theta_2) - 2*r_3*v_3*cos(theta_3) - r_1*u_1*sin(theta_1) - r_2*u_2*sin(theta_2) + 2*r_3*u_3*sin(theta_3);
            l_sigma*(lambda_sigma*y_1 + lambda_sigma*y_2 - 2*lambda_sigma*y_3 + v_1*cos(theta_1) + v_2*cos(theta_2) - 2*v_3*cos(theta_3) - u_1*sin(theta_1) - u_2*sin(theta_2) + 2*u_3*sin(theta_3)) + lambda_sigma*(v_1*cos(theta_1) - u_1*sin(theta_1)) + lambda_sigma*(v_2*cos(theta_2) - u_2*sin(theta_2)) - 2*lambda_sigma*(v_3*cos(theta_3) - u_3*sin(theta_3)) + v_1_dot*cos(theta_1) + v_2_dot*cos(theta_2) - u_1_dot*sin(theta_1) - u_2_dot*sin(theta_2) + k_sigma*tanh((lambda_sigma*y_1 + lambda_sigma*y_2 - 2*lambda_sigma*y_3 + v_1*cos(theta_1) + v_2*cos(theta_2) - 2*v_3*cos(theta_3) - u_1*sin(theta_1) - u_2*sin(theta_2) + 2*u_3*sin(theta_3))/epsilon_sigma) - r_1*u_1*cos(theta_1) - r_2*u_2*cos(theta_2) + 2*r_3*u_3*cos(theta_3) - r_1*v_1*sin(theta_1) - r_2*v_2*sin(theta_2) + 2*r_3*v_3*sin(theta_3);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    r_1_dot + r_2_dot + k_sigma*tanh((r_1 + r_2 - 2*r_3 + lambda_sigma*theta_1 + lambda_sigma*theta_2 - 2*lambda_sigma*theta_3)/epsilon_sigma) + lambda_sigma*r_1 + lambda_sigma*r_2 - 2*lambda_sigma*r_3 + l_sigma*(r_1 + r_2 - 2*r_3 + lambda_sigma*theta_1 + lambda_sigma*theta_2 - 2*lambda_sigma*theta_3);];
 
sigma_3=[2*lambda_sigma*x_3 - lambda_sigma*x_2 - lambda_sigma*(x_1 + 1) - u_1*cos(theta_1) - u_2*cos(theta_2) + 2*u_3*cos(theta_3) - v_1*sin(theta_1) - v_2*sin(theta_2) + 2*v_3*sin(theta_3);
      2*lambda_sigma*y_3 - lambda_sigma*y_2 - lambda_sigma*y_1 - v_1*cos(theta_1) - v_2*cos(theta_2) + 2*v_3*cos(theta_3) + u_1*sin(theta_1) + u_2*sin(theta_2) - 2*u_3*sin(theta_3);
                                                                                            2*r_3 - r_2 - r_1 - lambda_sigma*theta_1 - lambda_sigma*theta_2 + 2*lambda_sigma*theta_3;];
 
%% REFERENCE MODEL

[nu,R,M,tau]=UGV_dynamics(x_3,y_3,theta_3,u_3,v_3,r_3);

%% TRAJECTORY GENERATOR

[x_SS_3_dot,Q_3]=cooperative_filter(A_c,b_c,nu,R,M,tau);

end
