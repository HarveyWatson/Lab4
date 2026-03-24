clc;clear; close all;

g = 9.81;
m = 0.068;
w = m*g;
I = [5.8e-5, 0, 0;
    0, 7.2e-5, 0;
    0, 0, 1e-4];
d = 0.06;
km = 0.0024;
nu = 1e-3;
mu = 2e-6;

Va = 5;
D = (Va^2)*nu;

% Trim variables 5 m/s East, yaw 0 deg (for yaw 90 deg, same value of phi is applied but just to the pitch)
phi = acot(w/D);

Zc = w / cos(phi);

vE = (w*sin(phi))/(D/Va);
wE = (1/(D/Va))*(w*cos(phi)-Zc);
% end

% steady hovering flight conditions
Zc = w;
% end
tspan = [0 10];
var0 = [0, 0, -5, 0, 0, 0, 0, 2, 0, 0.02, 0.04, 0]; % given random perturbations as initial conditions

motor_forces = [Zc/4, Zc/4, Zc/4, Zc/4];

[t, var] = ode45(@(t, var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces), tspan, var0);

% Verify perturbations make quadrotor unstable (no control)
load("RSdata_nocontrol.mat");

times = rt_estim.time(:);
xdata = rt_estim.signals.values(:,1);
control_force_array = zeros(4,length(t));
control_force_array(1,:) = -w;
% plot sim
PlotAircraftSim(t, var', control_force_array, [1001, 1002, 1003, 1004, 1005, 1006]', 'b-');

control_force_array = zeros(4,length(times));
control_force_array(1,:) = -w;

% plot .mat file
PlotAircraftSim(times, rt_estim.signals.values', control_force_array, [2001, 2002, 2003, 2004, 2005, 2006]', 'b-');

