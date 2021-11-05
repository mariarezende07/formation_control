%% Initialization
n_agents = 5;

A=[
    [0,0,0,0,0];
    [1,0,1,0,1];
    [1,1,0,1,0];
    [1,0,1,0,1];
    [1,1,0,1,0]
    ];

D=[
    [0,0,0,0,0];
    [0,3,0,0,0];
    [0,0,3,0,0];
    [0,0,0,3,0];
    [0,0,0,0,3]
    ];

A = sym(A);
D = sym(D);

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

syms lambda_sigma k_sigma l_sigma epsilon_sigma
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
delta_x = sym('delta_x_%d_%d',[n_agents n_agents]);
delta_y = sym('delta_y_%d_%d',[n_agents n_agents]);
delta_theta = sym('delta_theta_%d_%d',[n_agents n_agents]);

delta{n_agents,n_agents} = [n_agents:n_agents];

for i=1:n_agents
    for j=1:n_agents    
    delta{i,j} = [delta_x(i,j);delta_y(i,j);delta_theta(i,j)];
    end
end


eta_dot = sym(eta_dot);
eta = sym(eta);
delta = sym(delta);

% %% Sigma
sigma = sym(zeros(length(eta(:,1)),n_agents));

for i=1:n_agents
    
    for j=1:n_agents
    
        sigma(:,i) = sigma(:,i) - A(i,j) * (eta_dot(:,j) + lambda_sigma * (eta(:,j) + delta(i,j) ));

    end
    
    sigma(:,i) = D(i,i) * (eta_dot(:,i) + lambda_sigma * eta(:,i)) + sigma(:,i);

end

sigma=simplify(sigma);

%% Differential constraints
sigma_dot = sym(zeros(size(sigma)));

for i=1:n_agents
    sigma_dot(:,i) = diff(sigma(:,i),'t');
end

sigma_dot = subs(sigma_dot, diff(eta,'t'),eta_dot);

%% Formation
delta_x_f=[
    [0,0,0,0,0];
    [-1,0,-1,0,-1];
    [0,1,0,-1,0];
    [1,0,1,0,1];
    [0,1,0,-1,0]
    ];

delta_y_f=[
    [0,0,0,0,0];
    [0,0,1,0,-1];
    [-1,-1,0,-1,0];
    [0,0,1,0,-1];
    [1,1,0,1,0]
    ];

delta_theta_f=zeros(size(A));

for i=1:n_agents
    for j=1:n_agents
        delta_x(i,j) = delta_x_f(i,j);
        delta_y(i,j) = delta_x_f(i,j);
        delta_theta(i,j) = delta_x_f(i,j);    
    end 
end
