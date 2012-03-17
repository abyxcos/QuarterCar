% Init Half Car Parameters
% This script initializes the global variables used in the half car model.

% Half-car Masses
h_car.ms = 840/2;    % Body mass
h_car.m1 = 53;       % Tire mass
h_car.m2 = 53;       % Tire mass
h_car.Ix = 820;      % Body inertia

% Body lengths
h_car.b1 = 0.7;
h_car.b2 = 0.7;
%h_car.b2 = 0.75;

% Springs
h_car.ku = 10000;
h_car.kt = 200000;

% Dampers
h_car.bu = 4800;     % Shocks

% Gravity
h_car.grav = 9.81;
