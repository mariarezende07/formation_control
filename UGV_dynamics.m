function [nu,R,M,tau]=UGV_dynamics(x,y,theta,u,v,r)

%% PARAMETERS

x_g=0;
m=275;
I_z=136.18;

X_u_dot=-488.12;
Y_v_dot=-787.28;
Y_r_dot=0;
N_r_dot=-48.25;

X_u=-74.77;
X_u_abs_u=-18.03;
Y_v=-134.86;
Y_v_abs_v=-34.05;
N_r=-2.36;
N_r_abs_r=-227.92;


%% MODEL

nu=[u;v;r];

R=[
[cos(theta),-sin(theta),0]
[sin(theta),cos(theta),0]
[0,0,1]
];

M=[
[m-X_u_dot,0,0];
[0,m-Y_v_dot,m*x_g-Y_r_dot];
[0,m*x_g-Y_r_dot,I_z-N_r_dot]
];

tau=[
u*(X_u+X_u_abs_u*abs(u))-r*(Y_r_dot*r+Y_v_dot*v-m*r*x_g)+m*r*v;
v*(Y_v+Y_v_abs_v*abs(v))+X_u_dot*r*u-m*r*u;
r*(N_r+N_r_abs_r*abs(r))+u*(Y_r_dot*r+Y_v_dot*v-m*r*x_g)-X_u_dot*u*v
];
