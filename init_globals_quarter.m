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

% Gravity
q_car.grav = 9.81;
