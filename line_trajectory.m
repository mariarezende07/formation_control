function eta_desired=line_trajectory(t,x_SS_alpha,x_0_points,x_T_points,y_0_points,y_T_points,theta_0_points,theta_T_points,T,psi_trajectory)

if t <= T
    
    x_desired=cycloid(x_0_points(1),x_T_points(1),T,t,0);
    y_desired=cycloid(y_0_points(1),y_T_points(1),T,t,0);
    theta_desired=cycloid(theta_0_points(1),theta_T_points(1),T,t,0);

elseif t > T && t <= 2*T
    
    x_desired=cycloid(x_0_points(2),x_T_points(2),T,t,T);
    y_desired=cycloid(y_0_points(2),y_T_points(2),T,t,T);
    theta_desired=cycloid(theta_0_points(2),theta_T_points(2),T,t,T);
    
elseif t > 2*T && t <= 3*T
    
    x_desired=cycloid(x_0_points(3),x_T_points(3),T,t,2*T);
    y_desired=cycloid(y_0_points(3),y_T_points(3),T,t,2*T);
    theta_desired=cycloid(theta_0_points(3),theta_T_points(3),T,t,2*T);
    
elseif t > 3*T && t <= 4*T
    
    x_desired=cycloid(x_0_points(4),x_T_points(4),T,t,3*T);
    y_desired=cycloid(y_0_points(4),y_T_points(4),T,t,3*T);
    theta_desired=cycloid(theta_0_points(4),theta_T_points(4),T,t,3*T);
    
elseif t > 4*T && t <= 5*T
    
    x_desired=cycloid(x_0_points(5),x_T_points(5),T,t,4*T);
    y_desired=cycloid(y_0_points(5),y_T_points(5),T,t,4*T);
    theta_desired=cycloid(theta_0_points(5),theta_T_points(5),T,t,4*T);
    
else     
    x_desired=cycloid(x_0_points(6),x_T_points(6),T,t,5*T);
    y_desired=cycloid(y_0_points(6),y_T_points(6),T,t,5*T);
    theta_desired=cycloid(theta_0_points(6),theta_T_points(6),T,t,5*T);    
    
end

eta_desired=[x_desired;y_desired;theta_desired];

alfa=psi_trajectory;

R=[
    [cos(alfa),-sin(alfa),0];
    [sin(alfa),cos(alfa),0];
    [0,0,1]
    ];

eta_desired=R*eta_desired;
eta_desired(3)=x_SS_alpha(1);


end