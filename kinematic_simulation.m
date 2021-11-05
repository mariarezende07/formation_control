clear all; clc; close all;
%% Parameters 
dt = 0.1; % Step size
ts = 10; % Simulation time
t = 0:dt:ts; % Time span


%% Initial conditions
x0 = 0.5;
y0=0.5;
psi0 = pi/4;

eta0=[x0;y0;psi0];
eta(:,1) = eta0;


for i = 1:length(t)
    psi = eta(3,i); % current orientation in rad

    J_psi = [cos(psi), -sin(psi),0; 
              sin(psi), cos(psi), 0;
              0, 0 , 1];

%% Desired states (Generalized coordinates)
    eta_d = [2-2*cos(0.1*t(i));2*sin(0.1*t(i));0.1*t(i)];
    eta_d_dot = [2*0.1*sin(0.1*t(i));2*0.1*cos(0.1*t(i));0.1];

    %% Vector of velocity input commands
    zeta(:,i) = inv(J_psi) * eta_d_dot;

    % Time derivative of generalized coordinates 
    eta_dot(:,i) = J_psi * zeta(:,i);

%% Position propagation using Euler method
    eta(:,i+1) = eta(:,i) + dt*eta_dot(:,i); % State update
end     
%% Mobile robot animation
l = 0.6; %length of the mobile robot
w = 0.4; %width of the mobile robot

mr_co = [-l/2, l/2, l/2, -l/2, -l/2;
         -w/2, -w/2, w/2, w/2, -w/2;];

figure
for i = 1:length(t)
    psi = eta(3,i); % current orientation in rad
    R_psi = [cos(psi), -sin(psi); 
               sin(psi), cos(psi);];
    v_pos = R_psi * mr_co;

    fill(v_pos(1,:)+eta(1,i),v_pos(2,:)+eta(2,i),'g')
    hold on, grid on; 
    axis([-1 3 -1 3]), axis square
    plot(eta(1,1:i),eta(2,1:i),'b-');
    legend('MR', 'Path'), set(gca, 'fontsize',24)
    xlabel('x[m]'); ylabel('y[m]');
    pause(0.1); hold off
end     
