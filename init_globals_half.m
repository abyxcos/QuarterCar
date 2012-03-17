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

% Mass matrix
h_car.m = [h_car.ms 0 0 0;
          0 h_car.Ix 0 0;
          0 0 h_car.m1 0;
          0 0 0 h_car.m2];

% Damper matrix
B11 = 2*h_car.bu;
B12 = h_car.bu*h_car.b1 - h_car.bu*h_car.b2;
B13 = -h_car.bu;
B14 = -h_car.bu;
B21 = h_car.bu*h_car.b1 - h_car.bu*h_car.b2;
B22 = h_car.bu*h_car.b1^2 + h_car.bu*h_car.b2^2;
B23 = -h_car.bu*h_car.b1;
B24 = h_car.bu*h_car.b2;
B31 = -h_car.bu;
B32 = -h_car.bu*h_car.b1;
B33 = h_car.bu;
B34 = 0;
B41 = -h_car.bu;
B42 = h_car.bu*h_car.b2;
B43 = 0;
B44 = h_car.bu;
h_car.b = [B11 B12 B13 B14;
          B21 B22 B23 B24;
          B31 B32 B33 B34;
          B41 B42 B43 B44];

% Spring matrix
K11 = 2*h_car.ku;
K12 = h_car.ku*h_car.b1 - h_car.ku*h_car.b2;
K13 = -h_car.ku;
K14 = -h_car.ku;
K21 = h_car.ku*h_car.b1 - h_car.ku*h_car.b2;
K22 = h_car.ku*h_car.b1^2 + h_car.ku*h_car.b2^2;
K23 = -h_car.ku*h_car.b1;
K24 = h_car.ku*h_car.b2;
K31 = -h_car.ku;
K32 = -h_car.ku*h_car.b1;
K33 = h_car.ku + h_car.kt;
K34 = 0;
K41 = -h_car.ku;
K42 = h_car.ku*h_car.b2;
K43 = 0;
K44 = h_car.ku + h_car.kt;
h_car.k = [K11 K12 K13 K14;
          K21 K22 K23 K24;
          K31 K32 K33 K34;
          K41 K42 K43 K44];
