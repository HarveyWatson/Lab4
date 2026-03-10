clc; 
clear; 
close;

%t, var, g, m, I, d, km, nu, mu, motor_forces;
tspan = 0:10;
var0 = [0, 0,-5,0,0,0,0,5,0,0,0,0]';
g= 9.81;
m = .068;
I = [(5.8*10^-5) 0 0; 0 (7.2*10^-5) 0; 0 0 (1*10^-4)];
d = .06;
km = 0.0024;
nu = 1*10^-3;
mu = 2*10^-6;

% grav force is f=ma=m*g, each motor force for trim is - m*g/4

IndvTrimF = (m*g)/4;
motor_forces = [IndvTrimF IndvTrimF IndvTrimF IndvTrimF]';

[t,var] = ode45(@(t,var) QuadrotorEOM(t,var,g,m,I,d,km,nu,mu,motor_forces), tspan, var0);