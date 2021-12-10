clear all
close all
clc

global A0 a0 b tau eta delta uncertainty_on_off k
global x1 x2 lambda lambda1 lambda2 sigma1 sigma2

tau = 64.75;
mass = 1;
eta = 0.1;

a0 = 1 / tau;
A0 = -1.5;
b = 1 / mass;
delta = 0.1;
uncertainty_on_off = 1;
k = 2;

%adaptive control
x1 = 0.5;
x2 = -0.5;
lambda = 6;
lambda1 = 10;
lambda2 = 25;
sigma1 = 4;
sigma2 = 5;

% gamma가 0일때와 상수일때 비교해보자
gamma = 3;

r2d = 180/pi;

wn = 3;
zeta = 1;
