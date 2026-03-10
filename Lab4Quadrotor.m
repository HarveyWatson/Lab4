
clear;
clc;
close all;

% Matlab script to model quadcopter dynamics for 3801 Lab 4
%% Given Values
m = .068; %kg
d = .06; %m
km = .0024; %Nm/(N)
Imat = [.58e-5 0  0 ; 0 7.2e-5 0 ; 0 0 1e-4];%kgm^2
g = 9.81; %m/s^2
nu = 1e-3;
mu = 2e-6;

%% Read in
% Time Array
load("RSdata_nocontrol.mat");

times = rt_estim.time;


% Estimated State Matrix
for i=1:12
    for j=1:1208
        estimStateMat(j,i) = rt_estim.signals.values(j,i);
    end
end

% Motor omegas
for i=1:4
    for j=1:1208
    omegaMotor(j,i)=sqrt(13840.4*rt_motor.signals.values(j,i));
    end
end

% Command Matrix
for i=1:4
    for j=1:1208
        if i==1
            controlMat(j,i) = rt_cmd.signals.values(j,i+3);
        else
            controlMat(j,i) = rt_cmd.signals.values(j,i+4);
        end
    end
end


%% 2.2
motor_forces = -[m*g/4;m*g/4;m*g/4;m*g/4];
deltaFc = 0;
deltaGc=[0;0;0];
var1i = [0;0;-2.5;0;0;0;0;0;0;0;0;0];

for i=1:6
    if i<4
        var1i(3+i)=5;
    else
        var1i(6+i)=.1;
    end
 
[tnon1, varnon1] = ode45(@(tnon1,varnon1) QuadrotorEOM(tnon1, varnon1, g, m, Imat, d, km, nu, mu, motor_forces),[0,10],var1i);
controlforcesnon1 = zeros(4,length(tnon1));
controlforcesnon1(1,:) = -m*g;
PlotAircraftSim(tnon1, varnon1', controlforcesnon1, [1;2;3;4;5;6]+6*(i-1), 'b-',"2.1")

[t1,var1] = ode45(@(t1,var1) QuadrotorEOM_linearized(t1, var1, g, m, Imat, deltaFc, deltaGc), [0,10], var1i);
controlforces1 = zeros(4,length(t1));
controlforces1(1,:) = -m*g;
PlotAircraftSim(t1, var1', controlforces1 , [1;2;3;4;5;6]+6*(i-1), 'm--',"2.2")

end
 

%% Functions



% 2.2
function var_dot = QuadrotorEOM_linearized(t, var, g, m, I, deltaFc, deltaGc)

var_dot = zeros(12, 1);

x_E = var(1);
y_E = var(2);
z_E = var(3);
phi = var(4);
theta = var(5);
psi = var(6);
u_E = var(7);
v_E = var(8);
w_E = var(9);
p = var(10);
q = var(11);
r = var(12);

var_dot(1) = u_E;
var_dot(2) = v_E;
var_dot(3) = w_E;
var_dot(4) = p;
var_dot(5) = q;
var_dot(6) = r;
var_dot(7) = -g*theta;
var_dot(8) =  g*phi; 
var_dot(9) = deltaFc/m; 
var_dot(10) = deltaGc(1)/ I(1,1); 
var_dot(11) = deltaGc(2)/ I(2,2); 
var_dot(12) = deltaGc(3)/ I(3,3);

end

% 2.3
function motor_forces = ComputeMotorForces(Fc,Gc,d,km)
Aeroc = [Fc(3) ; Gc(1) ; Gc(2) ; Gc(3)];
motorForcesMat = [ -1 -1 -1 -1 ; -d/sqrt(2) -d/sqrt(2) d/sqrt(2) d/sqrt(2);...
     d/sqrt(2) -d/sqrt(2) -d/sqrt(2) d/sqrt(2) ; km -km km -km];
motor_forces = (motorForcesMat^-1*Aeroc')';
end

% 2.4
function [Fc, Gc] = RotationDerivativeFeedback(var, m, g)
Fc = [0; 0; m*g];
Gc = .0004.*[var(10);var(11);var(12)];
end



