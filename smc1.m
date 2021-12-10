function yout = smc1(input)

lambda = 20;
eta = 0.1;

xd = input(1);
xd_dot = input(2);
xd_ddot = input(3);
x1 = input(4);
x2 = input(5);

fhat = -1.5 * x2^2 * cos(3*x1);
F = 0.5 * x2^2 * abs(cos(3*x1));

x_tilde = x1 - xd;
x_tilde_dot = x2 - xd_dot;
s = x_tilde_dot + lambda * x_tilde;

if s > 0
    sgn_s = 1;
else
    sgn_s = -1;
end

% Smoothing for chattering 
% phi = 0.1;
% if s>phi
%     sgn_s = 1;
% elseif s < -phi
%     sgn_s = -1;
% else
%     sgn_s = s / phi;
% end

uhat = - fhat + xd_ddot - lambda * x_tilde_dot;
k = F + eta;

yout = uhat - k*sgn_s;
