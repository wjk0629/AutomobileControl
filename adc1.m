function yout = adc1(input)

lambda = 6;

xd = input(1);
xd_dot = input(2);
xd_ddot = input(3);
x = input(4);
xdot = input(5);

m_hat = input(6);

xtilde = x - xd;
xtilde_dot = xdot - xd_dot;

s = xtilde_dot + lambda * xtilde;
v = xd_ddot - 2*lambda*xtilde_dot - lambda^2*xtilde;

u = m_hat * v;

yout = [u; s*v];
