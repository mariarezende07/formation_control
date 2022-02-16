%% Initialization

clearvars
close all
clc

n_agents = 6;

A = [
        [0,0,0,0,0,0];
        [1,0,1,0,0,1];
        [1,1,0,1,0,0];
        [1,0,1,0,1,0];
        [1,0,0,1,0,1];
        [1,1,0,0,1,0];   
    ];

D = [
        [0,0,0,0,0,0];
        [0,3,0,0,0,0];
        [0,0,3,0,0,0];
        [0,0,0,3,0,0];
        [0,0,0,0,3,0];
        [0,0,0,0,0,3];       
    ];
A = sym(A);
D = sym(D);

syms t;
x = sprintfc('x_%d(t)', 1:n_agents);
x= str2sym(x);


y = sprintfc('y_%d(t)', 1:n_agents);
y = str2sym(y);

theta = sprintfc('theta_%d(t)', 1:n_agents);
theta = str2sym(theta);

u = sprintfc('u_%d(t)', 1:n_agents);
u= str2sym(u);


w = sprintfc('w_%d(t)', 1:n_agents);
w = str2sym(w);

r = sprintfc('r_%d(t)', 1:n_agents);
r = str2sym(r);

R{n_agents} = 1:n_agents;

for i=1:n_agents
	R{i} =[
		[cos(theta(i)), sin(theta(i)), 0];
		[-sin(theta(i)), cos(theta(i)), 0];
		[0,0,1];
	];
end

syms m lambda_sigma k_sigma l_sigma epsilon_sigma
%% Column vectors
eta{n_agents} = 1:n_agents;

for i=1:n_agents
	eta{i} =[x(i);y(i);theta(i)];
end

nu{n_agents} = 1:n_agents;

for i=1:n_agents
	nu{i} =[u(i);w(i);r(i)];
end


eta_dot{n_agents} = 1:n_agents;

for i=1:n_agents
	eta_dot{i} = R{i}*nu{i};
end

%% Delta
clear delta
delta_x = sym('delta_x_%d_%d',[n_agents n_agents]);
delta_y = sym('delta_y_%d_%d',[n_agents n_agents]);
delta_theta = sym('delta_theta_%d_%d',[n_agents n_agents]);

%delta = sym(zeros(length(eta(:,1)),n_agents));
for i=1:n_agents
    for j=1:n_agents
        delta(:,j,i) = [delta_x(i,j);delta_y(i,j);delta_theta(i,j)];
    end
end



eta_dot = sym(eta_dot);
eta = sym(eta);
delta = sym(delta);

%% Sigma
sigma = sym(zeros(length(eta(:,1)),n_agents));

for i=1:n_agents
    
    for j=1:n_agents
    
        sigma(:,i) = sigma(:,i) - A(i,j) * (eta_dot(:,j) + lambda_sigma * (eta(:,j) + delta(:,j,i) ));

    end
    
    sigma(:,i) = D(i,i) * (eta_dot(:,i) + lambda_sigma * eta(:,i)) + sigma(:,i);

end

sigma=simplify(sigma);

%% Differential constraints
sigma_dot = sym(zeros(size(sigma)));
for i=1:n_agents
    sigma_dot(:,i) = diff(sigma(:,i),t);
end
sigma_dot = subs(sigma_dot, diff(eta,t),eta_dot);

eta_ddot =  diff(eta_dot,t);
eta_ddot = simplify(eta_ddot);

%% 
x_syms = sprintfc('x_%d', 1:n_agents);
x_syms= str2sym(x_syms);

y_syms = sprintfc('y_%d', 1:n_agents);
y_syms= str2sym(y_syms);

theta_syms = sprintfc('theta_%d', 1:n_agents);
theta_syms= str2sym(theta_syms);

u_syms = sprintfc('u_%d', 1:n_agents);
u_syms= str2sym(u_syms);

v_syms = sprintfc('v_%d', 1:n_agents);
v_syms= str2sym(v_syms);

r_syms = sprintfc('r_%d', 1:n_agents);
r_syms= str2sym(r_syms);

u_dot_syms = sprintfc('u_%d_dot', 1:n_agents);
u_dot_syms= str2sym(u_dot_syms);

v_dot_syms = sprintfc('v_%d_dot', 1:n_agents);
v_dot_syms= str2sym(v_dot_syms);

r_dot_syms = sprintfc('r_%d_dot', 1:n_agents);
r_dot_syms= str2sym(r_dot_syms);

eta_syms{n_agents} = 1:n_agents;
nu_syms{n_agents} = 1:n_agents;
nu_dot_syms{n_agents} = 1:n_agents;

eta_full_syms = [];
nu_full_syms = [];
nu_dot_full_syms = [];

for i=1:n_agents
    
    eta_syms{i} = [x_syms(i); y_syms(i); theta_syms(i)];
    nu_syms{i} = [u_syms(i); v_syms(i); r_syms(i)];
    nu_dot_syms{i} = [u_dot_syms(i); v_dot_syms(i); r_dot_syms(i)];

    eta_full_syms=[eta_full_syms,eta_syms{i}];
    nu_full_syms=[nu_full_syms,nu_syms{i}];
    nu_dot_full_syms=[nu_dot_full_syms,nu_dot_syms{i}];
    
end
%%%%%%%%%
eta_ddot_syms = subs(eta_ddot, diff(eta,t),eta_dot);
sigma_syms=subs(sigma,[eta,nu,diff(nu,t)],[eta_full_syms,nu_full_syms,nu_dot_full_syms]);
sigma_dot_syms=subs(sigma_dot,[eta,nu,diff(nu,t)],[eta_full_syms,nu_full_syms,nu_dot_full_syms]);
sigma_dot_syms=simplify(sigma_dot_syms);



%% Matrices separation
A_c=sym(zeros(3,3,n_agents));
b_c=sym(zeros(3,n_agents));
verification=sym(zeros(3,n_agents));

for i=1:n_agents

    A_c(:,:,i) = jacobian(sigma_dot_syms(:,i),nu_dot_syms{:,i});
    
    b_c(:,i) = - (sigma_dot_syms(:,i) - A_c(:,:,i) * nu_dot_syms{:,i});
    
    verification(:,i) = sigma_dot_syms(:,i) - (A_c(:,:,i) * nu_dot_syms{:,i} - b_c(:,i));
    
end

A_c=simplify(A_c);

for i=1:n_agents    
    
    b_c(:,i) = b_c(:,i) - k_sigma * tanh(sigma_syms(:,i)/epsilon_sigma) - l_sigma * sigma_syms(:,i);
    
end

b_c=simplify(b_c);

%% DYNAMICS

x_g = sprintfc('x_g_%d', 1:n_agents);
x_g= str2sym(x_g);

y_g = sprintfc('y_g_%d', 1:n_agents);
y_g= str2sym(y_g);

I_z = sprintfc('I_z_%d', 1:n_agents);
I_z= str2sym(I_z);
    

M=sym(zeros(3,3,n_agents));

for i=1:n_agents

    M(:,:,i) = [m, 0, -y_g(i)*m;
                0, m, x_g(i)*m;
                -y_g(i)*m, x_g(i)*m, I_z(i)+m*(x_g(i)^2+y_g(i)^2);];


    ud = nu_syms{i}(1);
    vd = nu_syms{i}(2);
    rd = nu_syms{i}(3);

    n_vd = [-m*rd*(vd+x_g(i)*rd);
            m*rd*(ud-y_g(i)*rd);
            m*rd*(x_g(i)*ud+y_g(i)*vd);];
    
    tau(:,i) =  M(:,:,i) * inv(R{i}) * nu_dot_syms(i) + n_vd;
    tau(:,i) = simplify(tau(:,i));
end

%% TRAJECTORY GENERATOR

H_c=sym(zeros(3,3,n_agents));

for i=2:n_agents
    H_c(:,:,i) = A_c(:,:,i) * (M(:,:,i)^(-1/2));
end

%% Formation

delta_x_f = [[0,  0, -1,  -1,  1,  1];
    [0,  0, -1,  -1,  1, 1];
    [1,  1,  0,  0,  1,  1];
    [1,  1,  0,  0,   1,  1];
    [-1, -1,-1, -1,  0,  0];
    [-1, -1,-1, -1,  0,  0]];

delta_y_f=[
    [0,  -1, 0,  1,  1,  0];
    [1,  0,  1,  1,  1, 1];
    [0,  -1,  0,  1,  1,  0];
    [-1, -1,  -1,  0,   0,  -1];
    [-1, -1,-1, -1,  0,  -1];
    [0, -1, 0, -1,  1,  0]];


delta_theta_f=zeros(size(A));

% for i=1:n_agents
%     for j=1:n_agents
%         delta_x(i,j) = delta_x_f(i,j);
%         delta_y(i,j) = delta_x_f(i,j);
%         delta_theta(i,j) = delta_x_f(i,j);    
%     end 
% end

b_c=subs(b_c, [delta_x,delta_y,delta_theta], [delta_x_f,delta_y_f,delta_theta_f]);

sigma_syms=subs(sigma_syms, [delta_x,delta_y,delta_theta], [delta_x_f,delta_y_f,delta_theta_f]);
