clear all; clc; close all;
%% Parameters 
dt = 0.1; % Step size
ts = 10; % Simulation time
t = 0:dt:ts; % Time span


%% Initial position

eta_1(:,1)=[0;0;0];
eta_2(:,1)=[-1;0;0];
eta_3(:,1)=[0;-1;0];
eta_4(:,1)=[1;0;0];
eta_5(:,1)=[0;1;0];


for i = 1:length(t)
    psi = eta_1(3,i); % current orientation in rad

    J_psi = [cos(psi), -sin(psi),0; 
              sin(psi), cos(psi), 0;
              0, 0 , 1];

%% Desired states (Generalized coordinates)
    eta_1_d = [2-2*cos(0.1*t(i));2*sin(0.1*t(i));0.1*t(i)];
    eta_1_d_dot = [2*0.1*sin(0.1*t(i));2*0.1*cos(0.1*t(i));0.1];

    %% Vector of velocity input commands
    zeta_1(:,i) = inv(J_psi) * eta_1_d_dot;

    % Time derivative of generalized coordinates 
    eta_1_dot(:,i) = J_psi * zeta_1(:,i);

%% Position propagation using Euler method
    eta_1(:,i+1) = eta_1(:,i) + dt*eta_1_dot(:,i); % State update
%%%%%%%%%%%%%%%%
    psi = eta_2(3,i); 
    J_psi = [cos(psi), -sin(psi),0; 
            sin(psi), cos(psi), 0;
            0, 0 , 1];
    eta_2_d = [4-2*cos(0.1*t(i));2*sin(0.1*t(i));0.1*t(i)];
    eta_2_d_dot = [4*0.1*sin(0.1*t(i));2*0.1*cos(0.1*t(i));0.1];

    %% Vector of velocity input commands
    zeta_2(:,i) = inv(J_psi) * eta_2_d_dot;

    % Time derivative of generalized coordinates 
    eta_2_dot(:,i) = J_psi * zeta_2(:,i);

    %% Position propagation using Euler method
    eta_2(:,i+1) = eta_2(:,i) + dt*eta_2_dot(:,i); % State update
end     


%% Mobile robot animation
l = 0.6; %length of the mobile robot
w = 0.4; %width of the mobile robot

mr_co = [-l/2, l/2, l/2, -l/2, -l/2;
         -w/2, -w/2, w/2, w/2, -w/2;];

figure
for i = 1:length(t)
    psi_1 = eta_1(3,i); % current orientation in rad
    R_psi_1 = [cos(psi_1), -sin(psi_1); 
               sin(psi_1), cos(psi_1);];
    v_pos_1 = R_psi_1 * mr_co;

    psi_2 = eta_2(3,i); % current orientation in rad
    R_psi_2 = [cos(psi_2), -sin(psi_2); 
            sin(psi_2), cos(psi_2);];
    v_pos_2 = R_psi_2 * mr_co;

    fill(v_pos_1(1,:)+eta_1(1,i),v_pos_1(2,:)+eta_1(2,i),'b',
         v_pos_2(1,:)+eta_2(1,i),v_pos_2(2,:)+eta_2(2,i),'b'
        );
    hold on, grid on; 
    axis([-1 3 -1 3]), axis square
    plot(eta_1(1,1:i),eta_1(2,1:i),'b-');
    plot(eta_2(1,1:i),eta_2(2,1:i),'b-');
    legend('MR', 'Path'), set(gca, 'fontsize',24)
    xlabel('x[m]'); ylabel('y[m]');
    pause(0.1); hold off

end     
