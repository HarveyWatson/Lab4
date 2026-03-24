% Contributors: Harvey Watson
% Course Number: ASEN 3801
% File Name: InnerLoopFeedback
% Created: 3/24/26

function [Fc, Gc] = InnerLoopFeedback(var)
% placeholders for gain values below
k1_latroll = 3.481*10^-4;
k2_latroll = 2.594*10^-4;

k1_longpitch = 4.321*10^-4;
k2_longpitch = 3.22*10^-4;

% lat roll
p = var(10);
phi = var(4);

% long pitch
q = var(11);
theta = var(5);


mass = 0.068;
g = 9.81;
weight = mass*g;
Fc = [0, 0, -weight]';
Lc = -k1_latroll*p -k2_latroll*phi;
Mc = -k1_longpitch*q -k2_longpitch*theta;
Nc = 0;

Gc = [Lc, Mc, Nc]';
end