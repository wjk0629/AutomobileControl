function yout = NNadc_HW(input)

global A0 b tau k

global x1 x2 sigma1 sigma2

xd = input(1);
xd_dot = input(2);
x = input(3);
x_dot = input(4);
w = input(5:6);

err = x - xd;

phi1 = exp(-(x-x1)^2/sigma1^2);
phi2 = exp(-(x-x2)^2/sigma2^2);

phi = [phi1; phi2];

u = (-1.5 * x_dot^2 * cos(3*x) + xd_dot - k*err - w.'*phi)/b;
wdot = err * phi;
yout = [u; wdot];
