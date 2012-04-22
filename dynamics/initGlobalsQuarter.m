% Init Quarter Car Parameters
% This script initializes the global variables used in the quarter car
% model.

% Quarter-car Masses
q_car.ms = 375;     % Body mass
q_car.mu = 75;      % Tire mass

% Springs
q_car.ks = 35000;   % Shocks
q_car.ku = 193000;  % Tire

% Dampers
q_car.bs = 2400;    % Shocks
q_car.bu = 0;       % Tire

% Mass Matrix
q_car.m = [q_car.ms 0;
          0 q_car.mu];

% Damper matrix
q_car.b = [q_car.bs -q_car.bs;
          -q_car.bs (q_car.bs+q_car.bu)];

% Spring matrix
q_car.k = [q_car.ks -q_car.ks;
          -q_car.ks (q_car.ks+q_car.ku)];
