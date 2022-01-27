function xi=cycloid(xi_0,xi_T,T,t,t_0)

delta_time=t-t_0;
delta_xi=xi_T-xi_0;

xi=xi_0+((delta_time/T)-(1/(2*pi))*sin((2*pi/T)*delta_time))*delta_xi;

end