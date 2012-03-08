% Init Full Car Parameters
% This script initializes the global variables used in the full car model.

% Full Car Masses
f_car.mu = 840;
f_car.mf = 53;
f_car.mr = 76;
f_car.Ix = 820;
f_car.Iy = 1100;

% Lengths
f_car.a1 = 1.4;
f_car.a2 = 1.47;
f_car.b1 = 0.7;
f_car.b2 = 0.75;
f_car.w = f_car.b1 + f_car.b2;

% Springs
f_car.kf = 10000;
f_car.kr = 13000;
f_car.ktf = 200000;
f_car.ktr = 200000;
f_car.kR = 0;

% Dampers
f_car.cf = 2000;
f_car.cr = 2000;

% Gravity
f_car.grav = 9.81;
