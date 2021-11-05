%% Parameters 
dt = 0.1; % Step size
ts = 10; % Simulation time
t = 0:dt:ts; % Time span

%% Trajectory
sliding_variable

%% Initial conditions
x_1_0=[[0;0;0];[0;0;0]];
x_2_0=[[-1;0;0];[0;0;0]];
x_3_0=[[0;-1;0];[0;0;0]];
x_4_0=[[1;0;0];[0;0;0]];
x_5_0=[[0;1;0];[0;0;0]];

eta_1_0 = [x_1_0(1);x_1_0(2);x_1_0(3)];

for i = 1:length(t)
    psi = eta_1_(3); % current orientation in rad

    R_1_psi = [cos(psi), -sin(psi),0; 
              sin(psi), cos(psi), 0;
              0, 0 , 1];
    u = 0.1; % x-axis velocity B frame
    w = 0.5; % y-axis velocity B frame
    r = 0.3; % angular velocity B frame

    zeta(:,i) = [u;v;r];

    eta_dot(:,i) = J_psi * zeta(:,i);
    eta(:,i+1) = eta(:,i) + dt*eta_dot(:,i); % Euler-method
end     
%% Mobile robot animation
l = 0.6; %length of the mobile robot
w = 0.4; %width of the mobile robot

mr_co = [-l/2, l/2, l/2, -l/2, -l/2;
         -w/2, -w/2, w/2, w/2, -w/2;];

figure
for j = n_agents
    for i = 1:length(t)
        psi = eta(3,i); % current orientation in rad

        R_psi = [cos(psi), -sin(psi); 
                sin(psi), cos(psi);];
        v_pos = R_psi * mr_co;

        fill(v_pos(1,:)+eta(1,i),v_pos(2,:)+eta(2,i),'g')
        hold on, grid on; axis([-1 3 -1 3]), axis square
        plot(eta(1,1:i),eta(2,1:i),'b-');
        legend('MR', 'Path'), set(gca, 'fontsize',24)
        xlabel('x[m]'); ylabel('y[m]');
        pause(0.1); hold off
    end     
end
