function eta_desired=line_trajectory(t, x_SS_alpha, x_SS_leader, target_point, obstacles)


x_desired= x_SS_leader(1) + (target_point(1) - x_SS_leader(1)) ;
y_desired=x_SS_leader(2) + (target_point(2) - x_SS_leader(2)) ;
theta_desired=0;

if obstacles(1) == x_desired
    y_desired = y_desided + 10;
end

eta_desired=[x_desired;y_desired;theta_desired];

alfa=0;

R=[
    [cos(alfa),-sin(alfa),0];
    [sin(alfa),cos(alfa),0];
    [0,0,1]
    ];

eta_desired=R*eta_desired;
eta_desired(3)=x_SS_alpha(1);


end
