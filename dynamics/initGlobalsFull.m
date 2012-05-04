% Init Full Car Parameters
% This script initializes the global variables used in the full car model.

% Full Car Masses
f_car.mu = 840;
f_car.mf = 53;
f_car.mr = 53;
%f_car.mr = 76;
f_car.Ix = 820;
f_car.Iy = 1100;

% Car speed
f_car.speed = 13;   % 13m/s ~~ 30 mph

% Lengths
f_car.a1 = 1.4;
f_car.a2 = 1.4;
%f_car.a2 = 1.47;
%f_car.back_lag = 0.1385;
f_car.back_lag = (f_car.a1+f_car.a2)/f_car.speed;
f_car.b1 = 0.7;
f_car.b2 = 0.7;
%f_car.b2 = 0.75;
f_car.w = f_car.b1 + f_car.b2;
f_car.turning_radius = 9;   % 30 ft turning radius

% Heights at rest
f_car.h_body = -0.2163;
f_car.h_wheel = -0.0103;

% Springs
f_car.kf = 10000;
f_car.kr = 10000;
%f_car.kr = 13000;
f_car.ktf = 200000;
f_car.ktr = 200000;
f_car.kR = 0;

% Dampers
f_car.cf = 9600;
f_car.cr = 9600;

% Mass matrix
f_car.m = [f_car.mu 0 0 0 0 0 0;
          0 f_car.Ix 0 0 0 0 0;
          0 0 f_car.Iy 0 0 0 0;
          0 0 0 f_car.mf 0 0 0;
          0 0 0 0 f_car.mf 0 0;
          0 0 0 0 0 f_car.mr 0;
          0 0 0 0 0 0 f_car.mr];

% Damper matrix
C11 = 2*f_car.cf + 2*f_car.cr;
C12 = f_car.b1*f_car.cf - f_car.b2*f_car.cf - f_car.b1*f_car.cr + ...
    f_car.b2*f_car.cr;
C13 = 2*f_car.a2*f_car.cr - 2*f_car.a1*f_car.cf;
C14 = -f_car.cf;
C15 = -f_car.cf;
C16 = -f_car.cr;
C17 = -f_car.cr;
C21 = C12;
C22 = f_car.b1^2*f_car.cf + f_car.b2^2*f_car.cf + f_car.b1^2*f_car.cr + ...
    f_car.b2^2*f_car.cr;
C23 = f_car.a1*f_car.b2*f_car.cf - f_car.a1*f_car.b1*f_car.cf - ...
    f_car.a2*f_car.b1*f_car.cr + f_car.a2*f_car.b2*f_car.cr;
C24 = -f_car.b1*f_car.cf;
C25 = f_car.b2*f_car.cf;
C26 = f_car.b1*f_car.cr;
C27 = -f_car.b2*f_car.cr;
C31 = C13;
C32 = C23;
C33 = 2*f_car.cf*f_car.a1^2 + 2*f_car.cr*f_car.a2^2;
C34 = f_car.a1*f_car.cf;
C35 = f_car.a1*f_car.cf;
C36 = -f_car.a2*f_car.cr;
C37 = -f_car.a2*f_car.cr;
C41 = -f_car.cf;
C42 = -f_car.b1*f_car.cf;
C43 = f_car.a1*f_car.cf;
C44 = f_car.cf;
C45 = 0;
C46 = 0;
C47 = 0;
C51 = -f_car.cf;
C52 = f_car.b2*f_car.cf;
C53 = f_car.a1*f_car.cf;
C54 = 0;
C55 = f_car.cf;
C56 = 0;
C57 = 0;
C61 = -f_car.cr;
C62 = f_car.b1*f_car.cr;
C63 = -f_car.a2*f_car.cr;
C64 = 0;
C65 = 0;
C66 = f_car.cr;
C67 = 0;
C71 = -f_car.cr;
C72 = -f_car.b2*f_car.cr;
C73 = -f_car.a2*f_car.cr;
C74 = 0;
C75 = 0;
C76 = 0;
C77 = f_car.cr;
f_car.b = [C11 C12 C13 C14 C15 C16 C17;
          C21 C22 C23 C24 C25 C26 C27;
          C31 C32 C33 C34 C35 C36 C37;
          C41 C42 C43 C44 C45 C46 C47;
          C51 C52 C53 C54 C55 C56 C57;
          C61 C62 C63 C64 C65 C66 C67;
          C71 C72 C73 C74 C75 C76 C77];

% Spring matrix
K11 = 2*f_car.kf + 2*f_car.kr;
K12 = f_car.b1*f_car.kf - f_car.b2*f_car.kf - f_car.b1*f_car.kr + ...
    f_car.b2*f_car.kr;
K13 = 2*f_car.a2*f_car.kr - 2*f_car.a1*f_car.kf;
K14 = -f_car.kf;
K15 = -f_car.kf;
K16 = -f_car.kr;
K17 = -f_car.kr;
K21 = K12;
K22 = f_car.kR + f_car.b1^2*f_car.kf + f_car.b2^2*f_car.kf + ...
    f_car.b1^2*f_car.kr + f_car.b2^2*f_car.kr;
K23 = f_car.a1*f_car.b2*f_car.kf - f_car.a1*f_car.b1*f_car.kf - ...
    f_car.a2*f_car.b1*f_car.kr + f_car.a2*f_car.b2*f_car.kr;
K24 = -f_car.b1*f_car.kf - (1/f_car.w)*f_car.kR;
K25 = f_car.b2*f_car.kf + (1/f_car.w)*f_car.kR;
K26 = f_car.b1*f_car.kr;
K27 = -f_car.b2*f_car.kr;
K31 = K13;
K32 = K23;
K33 = 2*f_car.kf*f_car.a1^2 + 2*f_car.kr*f_car.a2^2;
K34 = f_car.a1*f_car.kf;
K35 = f_car.a1*f_car.kf;
K36 = -f_car.a2*f_car.kr;
K37 = -f_car.a2*f_car.kr;
K41 = -f_car.kf;
K42 = K24;
K43 = f_car.a1*f_car.kf;
K44 = f_car.kf + f_car.ktf + (1/(f_car.w^2))*f_car.kR;
K45 = -f_car.kR/(f_car.w^2);
K46 = 0;
K47 = 0;
K51 = -f_car.kf;
K52 = K25;
K53 = f_car.a1*f_car.kf;
K54 = -f_car.kR/(f_car.w^2);
K55 = f_car.kf + f_car.ktf + (1/(f_car.w^2))*f_car.kR;
K56 = 0;
K57 = 0;
K61 = -f_car.kr;
K62 = f_car.b1*f_car.kr;
K63 = -f_car.a2*f_car.kr;
K64 = 0;
K65 = 0;
K66 = f_car.kr + f_car.ktr;
K67 = 0;
K71 = -f_car.kr;
K72 = -f_car.b2*f_car.kr;
K73 = -f_car.a2*f_car.kr;
K74 = 0;
K75 = 0;
K76 = 0;
K77 = f_car.kr + f_car.ktr;
f_car.k = [K11 K12 K13 K14 K15 K16 K17;
           K21 K22 K23 K24 K25 K26 K27;
           K31 K32 K33 K34 K35 K36 K37;
           K41 K42 K43 K44 K45 K46 K47;
           K51 K52 K53 K54 K55 K56 K57;
           K61 K62 K63 K64 K65 K66 K67;
           K71 K72 K73 K74 K75 K76 K77];
