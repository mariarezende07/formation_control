function [nu,R,M,tau]=UGV_dynamics(x,y,theta,u,v,r)

% PARAMETERS

m = 10;
Iz = 0.1;
l = 0.6; %length of the mobile robot
w = 0.4; %width of the mobile robot
d = w/2;
xbc = 0; ybc= 0; % coordinates of mass center

%% MODEL

nu=[u;v;r];

R=[
[cos(theta),sin(theta),0]
[-sin(theta),cos(theta),0]
[0,0,1]
];

M = [m, 0, -ybc*m;
     0, m, xbc*m;
     -ybc*m, xbc*m, Iz+m*(xbc^2+ybc^2);];

tau=[-m*(v+xbc);
   m*(u-ybc);
   m*(xbc*u+ybc*v);];