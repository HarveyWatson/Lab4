% script to read in values for Task one functions


clear;
clc;
close all;

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
        AircraftStateArray(j,i) = rt_estim.signals.values(j,i);
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
            controlInputArray(j,i) = rt_cmd.signals.values(j,i+3);
        else
            controlInputArray(j,i) = rt_cmd.signals.values(j,i+4);
        end
    end
end


%% Derive Forces
% Motor Forces (F) Matrix
MotorForces = [m*g/4 m*g/4 m*g/4 m*g/4];
