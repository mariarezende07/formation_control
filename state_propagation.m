
R_agent{n_agents} = 1:n_agents;
x_SS_1 = x_SS_leader;
x_SS_1_0 = x_SS_leader(:,1);

for i=1:n_agents
    eval(['eta_',num2str(i),'(:,1) = x_SS_',num2str(i),'_0(1:3,1);']);
    eval(['zeta_',num2str(i),'(:,1) = x_SS_',num2str(i),'(4:6,1);']);
    
    theta = eval(['eta_',num2str(i),'(3)']);
	R_agent{i} =[
		[cos(theta), sin(theta), 0];
		[-sin(theta), cos(theta), 0];
		[0,0,1];
	];

    eval(['eta_',num2str(i),'_dot(:,1) = R_agent{',num2str(i),'}*zeta_',num2str(i),'(:,1)']);
    eval(['zeta_',num2str(i),'_dot(:,1) = [0;0;0]']);    

    m = 10;
    Iz = 0.1;
    l = 0.6; %length of the mobile robot
    w = 0.4; %width of the mobile robot
    d = w/2;
    xbc = 0; ybc= 0; % coordinates of mass center
    %% Inertia matrix
    M = [m, 0, -ybc*m;
         0, m, xbc*m;
         -ybc*m, xbc*m, Iz+m*(xbc^2+ybc^2);];

    for j=1:N-1

        u = eval(['zeta_',num2str(i),'(1,',num2str(j),')']);
        v = eval(['zeta_',num2str(i),'(2,',num2str(j),')']);
        r = eval(['zeta_',num2str(i),'(1,',num2str(j),')']);

        theta = eval(['eta_',num2str(i),'(3,',num2str(j),')']);

        R = [[cos(theta),-sin(theta),0];
        [sin(theta),cos(theta),0];
        [0,0,1];
        ];

        R_dot = [[-sin(theta)*r,-cos(theta)*r,0];
        [cos(theta)*r,-sin(theta)*r,0];
        [0,0,0];
        ];

        eta_dot_tilt = R * eval(['(zeta_',num2str(i),'(:,',num2str(j),') - x_SS_',num2str(i),'(4:6,',num2str(j),'));']);

        eta_tilt = eval(['eta_',num2str(i),'(:,',num2str(j),') - x_SS_',num2str(i),'(1:3,',num2str(j),')']);

        sigma = eta_dot_tilt + lambda_sigma * eta_tilt;

        A = R;

        b = -lambda_sigma*eta_dot_tilt - k_sigma*tanh(sigma);

        tau(:,j) = udwadia_kalaba_control(A, b, R, eval(['x_SS_',num2str(i),'(4:6,',num2str(j),')']));

        n_v = [-m*(v+xbc);
               m*(u-ybc);
               m*(xbc*u+ybc*v);];

        eval(['zeta_',num2str(i),'_dot(:,',num2str(j),'+1) = M^(-1)*(tau(:,j)- n_v);']);
        eval(['zeta_',num2str(i),'(:,',num2str(j),'+1) = zeta_',num2str(i),'(:,',num2str(j),') + dt* zeta_',num2str(i),'_dot(:,',num2str(j),');']);

        eval(['eta_',num2str(i),'(:,',num2str(j),'+1) = eta_',num2str(i),'(:,',num2str(j),') + dt * R * zeta_',num2str(i),'(:,',num2str(j),');']);

    end
end