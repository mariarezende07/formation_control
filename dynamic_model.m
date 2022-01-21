%% Robot parameters
m = 10;
Iz = 0.1;
l = 0.6; %length of the mobile robot
w = 0.4; %width of the mobile robot
d = w/2;
xbc = 0; ybc= 0; % coordinates of mass center

t = 10;
%% State propagation

for i = 1:length(t)
    u = zeta(1,i);
    v = zeta(2,i);
    r = zeta(3,i);

    %% Inertia matrix
    D = [m, 0, -ybc*m;
         0, m, xbc*m;
         -ybc*m, xbc*m, Iz+m*(xbc^2+ybc^2);];
    n_v = [-m*r*(v+xbc*r);
           m*r*(u-ybc*r);
           m*r*(xbc*u+ybc*v);];
    %% Wheel input matrix (Diff drive)
    Gamma = 1/sqrt(2) * [1,1,1,1;
    1,-1,1,-1;
    l-d, l-d, -(l-d), -(l-d)];
    %% Given trajectory

    eta_d(:,i) = [0.2*t(i)^2; 0.05*t(i)^2;0];
    eta_dot_d = [0.2*t(i); 0.05*t(i);0];
    eta_ddot_d = [0.2;0.05;0];

    %% J matrix based on desired
    theta = eta_d(3,i);
    J_etad = [
    [cos(theta),-sin(theta),0];
    [sin(theta),cos(theta),0];
    [0,0,1];
    ];
    zeta_d = inv(J_etad)*eta_dot_d;
    ud = zeta_d(1);
    vd = zeta_d(2);
    rd = zeta_d(3);

    n_vd = [-m*rd*(vd+xbc*rd);
    m*rd*(u-ybc*rd);
    m*rd*(xbc*ud+ybc*vd);];
    
    
    %% Input vector
    tau(:,i) =  D * inv(J_etad) * (eta_ddot_d) + n_vd
    zeta_dot(:,i) = inv(D) * (tau(:,i)-n_v - 0.3*zeta(:,i));
    zeta(:,i+1) = zeta(:,i) + dt* zeta_dot(:,i); % Velocity update

    eta(:,i+1) = eta(:,i) + dt * (J_eta *zeta(:,i)+dt*zeta_dot(:,i)); % State update

end

%% Mobile robot animation


mr_co = [-l/2, l/2, l/2, -l/2, -l/2;
         -w/2, -w/2, w/2, w/2, -w/2;];
