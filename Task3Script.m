%% Linearized EOM Closed Loop Response (no k3)
clc;clear; close all;

g = 9.81;
m = 0.068;
w = m*g;
I = [5.8e-5, 0, 0;
    0, 7.2e-5, 0;
    0, 0, 1e-4];

tspan = [0 10];
% perturbations can be changed below in var0
var0 = [0, 0, -2.5, 0, 0, 0, 0, 0, 0, 0, 0, 0];

[t, var] = ode45(@(t, var) linearizedEOM_CL(t, var, g, m, I), tspan, var0);

control_force_array = zeros(4,length(t));
control_force_array(1,:) = -w;
PlotAircraftSim(t, var', control_force_array, [1,2,3,4,5,6]','b-');

%% Non linear EOM Closed Loop Response (no k3)

clc;clear; close all;

g = 9.81;
m = 0.068;
w = m*g;
I = [5.8e-5, 0, 0;
    0, 7.2e-5, 0;
    0, 0, 1e-4];
nu = 1e-3;
mu = 2e-6;

tspan = [0 10];
% perturbations can be changed below in var0
var0 = [0, 0, -2.5, deg2rad(10), 0, 0, 0, 0, 0, 0, 0, 0];

[t, var] = ode45(@(t, var) QuadrotorEOM_CL(t, var, g, m, I, nu, mu), tspan, var0);

control_force_array = zeros(4,length(t));
control_force_array(1,:) = -w;
PlotAircraftSim(t, var', control_force_array, [1,2,3,4,5,6]','b-');