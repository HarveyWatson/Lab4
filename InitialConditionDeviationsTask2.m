clc; close all; clear all;

%t, var, g, m, I, d, km, nu, mu, motor_forces;
tspan = 0:10; %setting 10 second sim time

% var = {x, y, z, phi, theta, psi, u, v, w, p, q, r }
var0 = [0, 0,-2.5,0,0,5,0,0,0,0,0,0]'; %initial conditions (notably, including one deviation)

g= 9.81; %grav
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
var = var';
fig = [1, 2, 3, 4, 5, 6];
col = "b-";
%Plotting aircraft sim for FIRST DEVIATION: Roll
PlotAircraftSim(t, var, motor_forces,fig, col)