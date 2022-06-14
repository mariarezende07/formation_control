function [x_SS_dot,Q_c]=cooperative_filter(A_c,b_c,nu,R,M,f)

H_c = A_c * sqrt(inv(M));

H_c_PI = (H_c.') * (inv(H_c * (H_c.')));

Q_c = sqrt(M) * H_c_PI * (b_c - A_c * (inv(M)) * f);

x_SS_dot = [
  R * nu;
  (inv(M)) * (f + Q_c)
  ];
