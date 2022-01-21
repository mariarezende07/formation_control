function [nu,R,M,tau]=UGV_dynamics(x,y,theta,u,v,r)

%% PARAMETERS

x_g=0;
y_g=0;
m=275;
I_z=136.18;

u_dot=-488.12;
v_dot=-787.28;
r_dot=-48.25;


%% MODEL

nu=[u;v;r];

R=[
[cos(theta),-sin(theta),0]
[sin(theta),cos(theta),0]
[0,0,1]
];

M=[m, 0, -y_g*m;
0, m, x_g*m;
-y_g*m, x_g*m, I_z+m*(x_g^2+y_g^2);];

tau =  [-m*x_g*r^2-m*v*r+m*u_dot*cos(theta)-m*v_dot*sin(theta)-m*r_dot*y_g;
        -m*y_g*r^2+m*u*r+m*v_dot*cos(theta)+m*u_dot*sin(theta)+m*r_dot*x_g;
        I_z*r_dot+m*r_dot*x_g^2+m*r_dot*y_g^2-m*u_dot*y_g*cos(theta)+m*v_dot*x_g*cos(theta)+m*u_dot*x_g*sin(theta)+m*v_dot*y_g*sin(theta)+m*r*u*x_g+m*r*v*y_g;];


