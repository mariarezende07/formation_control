function eta_desired=circle_trajectory(t,x_SS_alpha,x_0_points,y_0_points,psi_trajectory)

    x_desired = 10*cos(t)+ x_0_points;
    y_desired = 10*sin(t)+ y_0_points;
    theta_desired = 0;    
    
    
    eta_desired=[x_desired;y_desired;theta_desired];
    
    alfa=psi_trajectory;
    
    R=[
        [cos(alfa),sin(alfa),0];
        [-sin(alfa),cos(alfa),0];
        [0,0,1]
        ];
    
    eta_desired=R*eta_desired;
    eta_desired(3)=x_SS_alpha(1);
    
    
    end