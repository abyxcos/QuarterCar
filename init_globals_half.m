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

% Mass matrix
h_car.m = [h_car.ms 0 0 0;
          0 h_car.Ix 0 0;
          0 0 h_car.m1 0;
          0 0 0 h_car.m2];

% Damper matrix
h_car.B11 = 2*h_car.bu;
h_car.B12 = h_car.bu*h_car.b1 - h_car.bu*h_car.b2;
h_car.B13 = -h_car.bu;
h_car.B14 = -h_car.bu;
h_car.B21 = h_car.bu*h_car.b1 - h_car.bu*h_car.b2;
h_car.B22 = h_car.bu*h_car.b1^2 + h_car.bu*h_car.b2^2;
h_car.B23 = -h_car.bu*h_car.b1;
h_car.B24 = h_car.bu*h_car.b2;
h_car.B31 = -h_car.bu;
h_car.B32 = -h_car.bu*h_car.b1;
h_car.B33 = h_car.bu;
h_car.B34 = 0;
h_car.B41 = -h_car.bu;
h_car.B42 = h_car.bu*h_car.b2;
h_car.B43 = 0;
h_car.B44 = h_car.bu;
h_car.b = [h_car.B11 h_car.B12 h_car.B13 h_car.B14;
          h_car.B21 h_car.B22 h_car.B23 h_car.B24;
          h_car.B31 h_car.B32 h_car.B33 h_car.B34;
          h_car.B41 h_car.B42 h_car.B43 h_car.B44];

% Spring matrix
h_car.K11 = 2*h_car.ku;
h_car.K12 = h_car.ku*h_car.b1 - h_car.ku*h_car.b2;
h_car.K13 = -h_car.ku;
h_car.K14 = -h_car.ku;
h_car.K21 = h_car.ku*h_car.b1 - h_car.ku*h_car.b2;
h_car.K22 = h_car.ku*h_car.b1^2 + h_car.ku*h_car.b2^2;
h_car.K23 = -h_car.ku*h_car.b1;
h_car.K24 = h_car.ku*h_car.b2;
h_car.K31 = -h_car.ku;
h_car.K32 = -h_car.ku*h_car.b1;
h_car.K33 = h_car.ku + h_car.kt;
h_car.K34 = 0;
h_car.K41 = -h_car.ku;
h_car.K42 = h_car.ku*h_car.b2;
h_car.K43 = 0;
h_car.K44 = h_car.ku + h_car.kt;
h_car.k = [h_car.K11 h_car.K12 h_car.K13 h_car.K14;
          h_car.K21 h_car.K22 h_car.K23 h_car.K24;
          h_car.K31 h_car.K32 h_car.K33 h_car.K34;
          h_car.K41 h_car.K42 h_car.K43 h_car.K44];
