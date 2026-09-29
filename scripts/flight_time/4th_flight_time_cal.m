clear all
clc


%% Load parameters
m=1.762;                % kg
rho=1.2;                % kg/m^3
g=9.81;                 % m/s^2
R=0.119;                % m
FoM= 0.8;
Cap = 1500;             % Capacity (mAh)
VBat= 14.8;             % Voltage Level (V)



%% Thrust of one propeller
Tprop= 1/4*m*g          % N



%% Ideal power from momentum theory
% propeller disc
Ap = R^2*pi;

% ideal power at hover
Pideal = Tprop^(3/2)/(sqrt(2*rho*Ap)) % Watt



%% real power
% Power at one propeller
Preal = Pideal/FoM
% Power for quadrotor
Pt = 4*Preal



%% Flight time
% Battery energy
Ebat = VBat*Cap/1000    % Wh

% flight time until battery is empty
time = Ebat/Pt          % hour

% time in minutes
timemin= time*60        % min
