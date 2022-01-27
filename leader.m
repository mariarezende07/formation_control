function x_SS_1_dot=leader(x_SS_1,eta_desired)

%% INPUTS

x_1=x_SS_1(1);
y_1=x_SS_1(2);
psi_1=x_SS_1(3);

u_1=x_SS_1(4);
v_1=x_SS_1(5);
r_1=x_SS_1(6);

%% LEADER MODEL

x_SS_1_dot=leader_model(x_1,y_1,psi_1,u_1,v_1,r_1,eta_desired);

end
