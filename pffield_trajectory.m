function x_SS_leader_dot=pffield_trajectory(x_SS_leader, target_point, obstacles,R)

x_target = target_point(1);
x_leader = x_SS_leader(1);
x_obstacle = obstacles(1);

y_target = target_point(2);
y_leader = x_SS_leader(2);
y_obstacle = obstacles(2);

x_dot = x_SS_leader(4);
y_dot = x_SS_leader(5);
theta_dot = x_SS_leader(6);

if (sqrt((x_obstacle - x_leader)^2 + (y_obstacle - y_leader)^2) <= 7^2)
    F_x =x_target - x_leader + (x_leader - x_obstacle)/(((x_leader - x_obstacle)^2 + (y_leader - y_obstacle)^2)^2);
    F_y =y_target - y_leader + (y_leader - y_obstacle)/(((x_leader - x_obstacle)^2 + (y_leader - y_obstacle)^2)^2);
else
    F_x = (x_target - x_leader);

    F_y = (y_target - y_leader);


end

x_ddot = F_x;
y_ddot = F_y;
theta_ddot = (atan(F_x/F_y) - x_SS_leader(3));


x_SS_leader_dot=[    
    [x_ddot;y_ddot;theta_ddot];
    [0;0;0];
    ];

end
