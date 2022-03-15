function eta_desired=line_trajectory(t, x_SS_alpha, x_SS_leader, target_point, obstacles)

theta_desired=0;

[x_desired, y_desired] = shpath([[0,0,0,0,0,0,0,0,0,0];
                                 [0,0,0,0,0,0,0,0,0,0];
                                 [0,0,0,1,0,0,0,0,0,0];
                                 [0,0,0,0,0,0,0,0,0,0];
                                 [0,0,1,0,0,0,0,0,0,0];
                                 [0,0,0,0,0,0,0,0,0,0];
                                 [0,0,0,0,0,1,0,0,0,0];
                                 [0,0,0,0,0,0,0,0,1,0];
                                 [0,0,0,0,0,0,0,0,0,0];
                                 [0,0,0,1,0,0,0,0,0,0];],10,1,1,10);
eta_desired=[x_desired(1);y_desired(1);theta_desired];

alfa=0;

R=[
    [cos(alfa),-sin(alfa),0];
    [sin(alfa),cos(alfa),0];
    [0,0,1]
    ];

eta_desired=R*eta_desired;
eta_desired(3)=x_SS_alpha(1);


end
