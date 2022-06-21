
x_SS_leader = zeros(6,N);
x_SS_2 = zeros(6,N);
x_SS_3 = zeros(6,N);
x_SS_4 = zeros(6,N);
x_SS_5 = zeros(6,N);
x_SS_6 = zeros(6,N);
x_SS_7 = zeros(6,N);
eta_desired = zeros(3,N);

sigma_2 = zeros(3,N);
Q_2 = zeros(3,N);

sigma_3 = zeros(3,N);
Q_3 = zeros(3,N);

sigma_4 = zeros(3,N);
Q_4 = zeros(3,N);

sigma_5 = zeros(3,N);
Q_5 = zeros(3,N);

sigma_6 = zeros(3,N);
Q_6 = zeros(3,N);

sigma_7 = zeros(3,N);
Q_7 = zeros(3,N);

x_SS_alpha = [0;0];

x_SS_leader(:,1) = x_SS_leader_0;
x_SS_2(:,1) = x_SS_2_0;
x_SS_3(:,1) = x_SS_3_0;
x_SS_4(:,1) = x_SS_4_0;
x_SS_5(:,1) = x_SS_5_0;
x_SS_6(:,1) = x_SS_6_0;
x_SS_7(:,1) = x_SS_7_0;

for i=1:N-1
    %% Circular trajectory
    theta = x_SS_leader(3,i);
    R=[
    [cos(theta), sin(theta), 0];
    [-sin(theta), cos(theta), 0];
    [0,0,1];
    ];
    x_SS_leader_dot = pffield_trajectory(x_SS_leader(:,i), target_point, obstacles,R);
    x_SS_leader(:,i+1) = x_SS_leader(:,i) + dt * x_SS_leader_dot

    [sigma_2(:,i+1),Q_2(:,i+1),x_SS_2_dot] = subsystem_trajectory_1(2,x_SS_leader(:,i),x_SS_2(:,i),x_SS_3(:,i),...
                                                                    x_SS_4(:,i),x_SS_5(:,i),x_SS_6(:,i), x_SS_7(:,i),...
                                                                    lambda_sigma,k_sigma,l_sigma,epsilon_sigma);

    x_SS_2(:,i+1) = x_SS_2(:,i) + dt * x_SS_2_dot;
    
    [sigma_3(:,i+1),Q_3(:,i+1),x_SS_3_dot] = subsystem_trajectory_1(3,x_SS_leader(:,i),x_SS_2(:,i),x_SS_3(:,i),...
                                                                    x_SS_4(:,i),x_SS_5(:,i),x_SS_6(:,i), x_SS_7(:,i),...
                                                                    lambda_sigma,k_sigma,l_sigma,epsilon_sigma);

    x_SS_3(:,i+1) = x_SS_3(:,i) + dt * x_SS_3_dot;

    [sigma_4(:,i+1),Q_4(:,i+1),x_SS_4_dot] = subsystem_trajectory_1(4,x_SS_leader(:,i),x_SS_2(:,i),x_SS_3(:,i),...
                                                                    x_SS_4(:,i),x_SS_5(:,i),x_SS_6(:,i), x_SS_7(:,i),...
                                                                    lambda_sigma,k_sigma,l_sigma,epsilon_sigma);

    x_SS_4(:,i+1) = x_SS_4(:,i) + dt * x_SS_4_dot;
    
    [sigma_5(:,i+1),Q_5(:,i+1),x_SS_5_dot] = subsystem_trajectory_1(5,x_SS_leader(:,i),x_SS_2(:,i),x_SS_3(:,i),...
                                                                    x_SS_4(:,i),x_SS_5(:,i),x_SS_6(:,i), x_SS_7(:,i),...
                                                                    lambda_sigma,k_sigma,l_sigma,epsilon_sigma);

    x_SS_5(:,i+1) = x_SS_5(:,i) + dt * x_SS_5_dot;
    
    [sigma_6(:,i+1),Q_6(:,i+1),x_SS_6_dot] = subsystem_trajectory_1(6,x_SS_leader(:,i),x_SS_2(:,i),x_SS_3(:,i),...
                                                                    x_SS_4(:,i),x_SS_5(:,i),x_SS_6(:,i), x_SS_7(:,i),...
                                                                    lambda_sigma,k_sigma,l_sigma,epsilon_sigma);

    x_SS_6(:,i+1) = x_SS_6(:,i) + dt * x_SS_6_dot;
    
    [sigma_7(:,i+1),Q_7(:,i+1),x_SS_7_dot] = subsystem_trajectory_1(7,x_SS_leader(:,i),x_SS_2(:,i),x_SS_3(:,i),...
                                                                    x_SS_4(:,i),x_SS_5(:,i),x_SS_6(:,i), x_SS_7(:,i),...
                                                                    lambda_sigma,k_sigma,l_sigma,epsilon_sigma);

    x_SS_7(:,i+1) = x_SS_7(:,i) + dt * x_SS_7_dot;
end