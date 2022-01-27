function x_SS_dot=leader_model(x,y,psi,u,v,r,eta_desired)

%% PARAMETERS

Zeta=diag([1,1,1]);

T_x=1;
T_y=1;
T_psi=1;

Omega=diag([2*pi/T_x,2*pi/T_y,2*pi/T_psi]);

%% MODEL

nu=[u;v;r];

eta=[x;y;psi];

R=[
[cos(psi),-sin(psi),0]
[sin(psi),cos(psi),0]
[0,0,1]
];

R_dot=[
    [-sin(psi)*r,-cos(psi)*r,0];
    [cos(psi)*r,-sin(psi)*r,0];
    [0,0,0]
    ];

x_SS_dot=[
    R*nu;
    (R.')*(Omega.^2)*eta_desired-2*Zeta*Omega*nu-(R.')*(Omega.^2)*eta-(R.')*R_dot*nu
    ];

