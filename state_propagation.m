
eta_leader(:,1) = [0;0;0];
eta_2(:,1) = [0;2;0];
eta_3(:,1) = [1;1;0];
eta_4(:,1) = [1;-1;0];
eta_5(:,1) = [0;-2;0];
eta_6(:,1) = [-1;-1;0];
eta_7(:,1) = [-1;1;0];

lambda_sigma=0.1;
k_sigma=5;

m = 10;
Iz = 0.1;
l = 0.6; %length of the mobile robot
w = 0.4; %width of the mobile robot
d = w/2;
xbc = 0; ybc= 0; % coordinates of mass center
%% 
theta = x_SS_2(3,1);
R = [[cos(theta),-sin(theta),0];
[sin(theta),cos(theta),0];
[0,0,1];
];
%% Inertia matrix
M = [m, 0, -ybc*m;
     0, m, xbc*m;
     -ybc*m, xbc*m, Iz+m*(xbc^2+ybc^2);];

zeta_2(:,1) = x_SS_2(4:6,1);
zeta_2_dot(:,1) = [0;0;0]; % R.'(eta_2_dot - R_dot * zeta_2)
eta_2(:,1) = x_SS_2(1:3,1);
eta_2_dot(:,1) = R * zeta_2(:,1);

for i=1:N-1
    % 2 
    u = zeta_2(1,i);
    v = zeta_2(2,i);
    r = zeta_2(3,i);
    
    theta = eta_2(3,i);
    
    R = [[cos(theta),-sin(theta),0];
    [sin(theta),cos(theta),0];
    [0,0,1];
    ];

    R_dot = [[-sin(theta)*r,-cos(theta)*r,0];
    [cos(theta)*r,-sin(theta)*r,0];
    [0,0,0];
    ];
    
    eta_dot_tilt = R * (zeta_2(:,i) - x_SS_2(4:6,i));
    
    eta_tilt = eta_2(:,i) - x_SS_2(1:3,i);
    
    sigma = eta_dot_tilt + lambda_sigma * eta_tilt;
    
    A = R;
    
    x = 1;
    
    %b = zeta_2_dot(:,i)*x -R_dot*zeta_2(:,i)*x -lambda_sigma*R*eta_tilt*x -k_sigma*tanh(sigma);
    b = -lambda_sigma*eta_dot_tilt - k_sigma*tanh(sigma);
    
    tau(:,i) = udwadia_kalaba_control(A, b, R, x_SS_2(4:6,i));
    
    n_v = [-m*(v+xbc);
           m*(u-ybc);
           m*(xbc*u+ybc*v);];

    zeta_2_dot(:,i+1) = M^(-1)*(tau(:,i)- n_v);
    zeta_2(:,i+1) = zeta_2(:,i) + dt* zeta_2_dot(:,i);

    %eta_2_dot(:,i+1) = eta_2_dot(:,i) + dt * R * zeta_2_dot(:,i);
    eta_2(:,i+1) = eta_2(:,i) + dt * R * zeta_2(:,i);

end