function yout = sm_adc(input)

lambda = 6;
k = 1;

xd = input(1);
xd_dot = input(2);
xd_ddot = input(3);
x = input(4);
xdot = input(5);

m_hat = input(6);

xtilde = x - xd;
xtilde_dot = xdot - xd_dot;

s = xtilde_dot + lambda * xtilde;

% if s > 0
%     sgn_s = 1;
% else
%     sgn_s = -1;
% end

% Smoothing for chattering 
phi = 0.2;
if s>phi
    sgn_s = 1;
elseif s < -phi
    sgn_s = -1;
else
    sgn_s = s / phi;
end

v = xd_ddot - lambda*xtilde_dot - k*sgn_s;
u = m_hat * v;

yout = [u; s*v];
