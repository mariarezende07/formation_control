N = length(time_vector);

x_SS_leader = zeros(6,N);
x_SS_2 = zeros(6,N);
x_SS_3 = zeros(6,N);

eta_desired = zeros(3,N);

sigma_2 = zeros(3,N);
Q_2 = zeros(3,N);

sigma_3 = zeros(3,N);
Q_3 = zeros(3,N);

x_SS_alpha = [0;0];

x_SS_leader(:,1) = x_SS_leader_0;
x_SS_2(:,1) = x_SS_2_0;
x_SS_3(:,1) = x_SS_3_0;

for i=1:N-1
    %% Circular trajectory
    eta_desired(:,i) = circle_trajectory(time_vector(i),x_SS_alpha,x_0_points,y_0_points,psi_trajectory);

    x_SS_leader_dot = leader(x_SS_leader(:,i),eta_desired(:,i));
    x_SS_leader(:,i+1) = x_SS_leader(:,i) + dt * x_SS_leader_dot;

    [sigma_2(:,i+1),Q_2(:,i+1),x_SS_2_dot] = subsystem_trajectory_1(2,x_SS_leader(:,i),x_SS_2(:,i),x_SS_3(:,i),...
                                                                    lambda_sigma,k_sigma,l_sigma,epsilon_sigma);

    x_SS_2(:,i+1) = x_SS_2(:,i) + dt * x_SS_2_dot;
    
    [sigma_3(:,i+1),Q_3(:,i+1),x_SS_3_dot] = subsystem_trajectory_1(3,x_SS_leader(:,i),x_SS_2(:,i),x_SS_3(:,i),...
                                                                    lambda_sigma,k_sigma,l_sigma,epsilon_sigma);

    x_SS_3(:,i+1) = x_SS_3(:,i) + dt * x_SS_3_dot;
    
    
end