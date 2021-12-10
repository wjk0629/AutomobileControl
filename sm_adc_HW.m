function yout = sm_adc_HW(input)

global lambda eta

xd = input(1);
xd_dot = input(2);
xd_ddot = input(3);
x = input(4);
x_dot = input(5);

m_hat = input(6);

% �ǹ���1. �� a(t)�� 1.5�ΰ�?
fhat = -1.5 * x_dot^2 * cos(3*x);
% �ǹ���2. F�� �̹� �˰� �ִ� ���̶�� �ߴµ�,
%          �Ʒ� ���� �ý��۸� ���� ��� �ƴ� ���ϱ�?
F = 0.5 * x_dot^2 * abs(cos(3*x));

xtilde = x - xd;
xtilde_dot = x_dot - xd_dot;

s = xtilde_dot + lambda * xtilde;

% if s > 0
%     sgn_s = 1;
% else
%     sgn_s = -1;
% end

% Smoothing for chattering 
phi = 0.1;
if s>phi
    sgn_s = 1;
elseif s < -phi
    sgn_s = -1;
else
    sgn_s = s / phi;
end

%k = F*eta;
k = 1;

v = - fhat + xd_ddot - lambda*xtilde_dot - k*sgn_s;
u = m_hat * v;

yout = [u; s*v];
