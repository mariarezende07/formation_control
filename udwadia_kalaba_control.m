function tau=udwadia_kalaba_control(A, b, R_dot , eta_dot)
    %% Robot parameters
    m = 10;
    Iz = 0.1;
    l = 0.6; %length of the mobile robot
    w = 0.4; %width of the mobile robot
    d = w/2;
    xbc = 0; ybc= 0; % coordinates of mass center
    u = eta_dot(1);
    v = eta_dot(2);
    
    n_v = [-m*(v+xbc);
           m*(u-ybc);
           m*(xbc*u+ybc*v);];
       
    
    %% Inertia matrix
    M = [m, 0, -ybc*m;
         0, m, xbc*m;
         -ybc*m, xbc*m, Iz+m*(xbc^2+ybc^2);];
    
    f = M * eta_dot + n_v;
    
    a = M^(-1) * f;
    %% Input vector
    H_c = A * M ^(-1/2);
    %H_c_dagger = (H_c.') * (inv(H_c * (H_c.')));
    tau = M^(1/2) * pinv(H_c) * (b- A * a);
    

end
