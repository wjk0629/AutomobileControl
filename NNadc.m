function yout = NNadc(input)

global a0 b tau k

global x1 x2 sigma1 sigma2

xd = input(1);
xd_dot = input(2);
x = input(3);
w = input(4:5);

err = x - xd;

phi1 = exp(-(x-x1)^2/sigma1^2);
phi2 = exp(-(x-x2)^2/sigma2^2);

phi = [phi1; phi2];

u = (-a0*x + xd_dot - k*err - w.'*phi)/b;
wdot = err * phi;
yout = [u; wdot];
