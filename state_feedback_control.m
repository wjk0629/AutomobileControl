Vx = 30;

[A,B,C,D] = lateral_model(Vx);

P = [-5-3*1j -5+3*1j -7 -10];

K = place(A,B(:,1),P);

Ac = A-B(:,1)*K;
Bc = B(:,2);
Cc = eye(4);
Dc = 0;
sys = ss(Ac,Bc,Cc,Dc);
t = 0:0.01:20;
R = 1000;

T = floor(length(t)/20);
u = [zeros(1,T) (Vx/R)*ones(1,length(t)-T)]';
[y,t,x] = lsim(sys,u,t);