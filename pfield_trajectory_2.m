E = 1;
N = 1;
K = 1;
num_agents = 7;
obstacle_r = 5;
max_dist_leader = 2;
%% Calculo do potencial atrativo
syms x_target x_leader y_target y_leader x_obstacle y_obstacle t

p_g = sqrt((x_target - x_leader)^2 + (y_target - y_leader)^2);

VG = 0.5 * E * p_g^2;
%VG = 0.5 *p_g;


%% Calculo do potencial repulsivo
p_o_b = sqrt((x_obstacle - x_leader)^2 + (y_obstacle - y_leader)^2) ; % d entre lider e obstaculo

p_0 = obstacle_r + max_dist_leader;

VO = 0.5 * N * ((1/p_o_b))^2;
%VO = 0;
%VO = 20/p_o_b;
U_x = gradient(VO+VG,x_leader);
U_y = gradient(VO+VG,y_leader);

F_x = -K*(U_x);
F_y = -K*(U_y);

%F_x = 10*((10-x_leader)/p_g) - 20*((x_obstacle-x_leader)/p_o_b^3)
%F_y = 10*((10-y_leader)/p_g) - 20*((x_obstacle-y_leader)/p_o_b^3)




