E = 1;
N = 1;
K = 1;
num_agents = 6;
obstacle_r = 5;
max_dist_leader = 2;
%% Calculo do potencial atrativo
syms x_target x_leader y_target y_leader x_obstacle y_obstacle

p_g = sqrt((x_target - x_leader)^2 + (y_target - y_leader)^2);

VG = 0.5 * E *p_g^2;

FG_x = -K * gradient(VG,x_leader);
FG_y = -K * gradient(VG,y_leader);

%% Calculo do potencial repulsivo
p_o_b = sqrt((x_obstacle - x_leader)^2 + (y_obstacle - y_leader)^2) ; % d entre lider e obstaculo

p_0 = obstacle_r + max_dist_leader;

VO = 0.5 * N * ((1/p_o_b)-(1/p_0))^2 * p_g^num_agents;

FO_x = -K * gradient(VO,x_leader);
FO_y = -K * gradient(VO,y_leader);

