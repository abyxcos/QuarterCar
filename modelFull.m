% Full Car Model

% modelFull - Determines the new xdot values for the full car
% Parameters:
%     t - The current time
%     x - The current state of the model
%     p - The model parameters
% Returns:
%     The xdot for the model in its current state
function xdot = modelFull(t, x, p)

    % Mass matrix
    m = [p.mu 0 0 0 0 0 0;
        0 p.Ix 0 0 0 0 0;
        0 0 p.Iy 0 0 0 0;
        0 0 0 p.mf 0 0 0;
        0 0 0 0 p.mf 0 0;
        0 0 0 0 0 p.mr 0;
        0 0 0 0 0 0 p.mr];
    
    % Damper matrix
    C11 = 2*p.cf + 2*p.cr;
    C12 = p.b1*p.cf - p.b2*p.cf - p.b1*p.cr + p.b2*p.cr;
    C13 = 2*p.a2*p.cr - 2*p.a1*p.cf;
    C14 = -p.cf;
    C15 = -p.cf;
    C16 = -p.cr;
    C17 = -p.cr;
    C21 = C12;
    C22 = p.b1^2*p.cf + p.b2^2*p.cf + p.b1^2*p.cr + p.b2^2*p.cr;
    C23 = p.a1*p.b2*p.cf - p.a1*p.b1*p.cf - p.a2*p.b1*p.cr + p.a2*p.b2*p.cr;
    C24 = -p.b1*p.cf;
    C25 = p.b2*p.cf;
    C26 = p.b1*p.cr;
    C27 = -p.b2*p.cr;
    C31 = C13;
    C32 = C23;
    C33 = 2*p.cf*p.a1^2 + 2*p.cr*p.a2^2;
    C34 = p.a1*p.cf;
    C35 = p.a1*p.cf;
    C36 = -p.a2*p.cr;
    C37 = -p.a2*p.cr;
    C41 = -p.cf;
    C42 = -p.b1*p.cf;
    C43 = p.a1*p.cf;
    C44 = p.cf;
    C45 = 0;
    C46 = 0;
    C47 = 0;
    C51 = -p.cf;
    C52 = p.b2*p.cf;
    C53 = p.a1*p.cf;
    C54 = 0;
    C55 = p.cf;
    C56 = 0;
    C57 = 0;
    C61 = -p.cr;
    C62 = p.b1*p.cr;
    C63 = -p.a2*p.cr;
    C64 = 0;
    C65 = 0;
    C66 = p.cr;
    C67 = 0;
    C71 = -p.cr;
    C72 = -p.b2*p.cr;
    C73 = -p.a2*p.cr;
    C74 = 0;
    C75 = 0;
    C76 = 0;
    C77 = p.cr;
    b = [C11 C12 C13 C14 C15 C16 C17;
        C21 C22 C23 C24 C25 C26 C27;
        C31 C32 C33 C34 C35 C36 C37;
        C41 C42 C43 C44 C45 C46 C47;
        C51 C52 C53 C54 C55 C56 C57;
        C61 C62 C63 C64 C65 C66 C67;
        C71 C72 C73 C74 C75 C76 C77];
    
    % Spring matrix
    K11 = 2*p.kf + 2*p.kr;
    K12 = p.b1*p.kf - p.b2*p.kf - p.b1*p.kr + p.b2*p.kr;
    K13 = 2*p.a2*p.kr - 2*p.a1*p.kf;
    K14 = -p.kf;
    K15 = -p.kf;
    K16 = -p.kr;
    K17 = -p.kr;
    K21 = K12;
    K22 = p.kR + p.b1^2*p.kf + p.b2^2*p.kf + p.b1^2*p.kr + p.b2^2*p.kr;
    K23 = p.a1*p.b2*p.kf - p.a1*p.b1*p.kf - p.a2*p.b1*p.kr + p.a2*p.b2*p.kr;
    K24 = -p.b1*p.kf - (1/p.w)*p.kR;
    K25 = p.b2*p.kf + (1/p.w)*p.kR;
    K26 = p.b1*p.kr;
    K27 = -p.b2*p.kr;
    K31 = K13;
    K32 = K23;
    K33 = 2*p.kf*p.a1^2 + 2*p.kr*p.a2^2;
    K34 = p.a1*p.kf;
    K35 = p.a1*p.kf;
    K36 = -p.a2*p.kr;
    K37 = -p.a2*p.kr;
    K41 = -p.kf;
    K42 = K24;
    K43 = p.a1*p.kf;
    K44 = p.kf + p.ktf + (1/(p.w^2))*p.kR;
    K45 = -p.kR/(p.w^2);
    K46 = 0;
    K47 = 0;
    K51 = -p.kf;
    K52 = K25;
    K53 = p.a1*p.kf;
    K54 = -p.kR/(p.w^2);
    K55 = p.kf + p.ktf + (1/(p.w^2))*p.kR;
    K56 = 0;
    K57 = 0;
    K61 = -p.kr;
    K62 = p.b1*p.kr;
    K63 = -p.a2*p.kr;
    K64 = 0;
    K65 = 0;
    K66 = p.kr + p.ktr;
    K67 = 0;
    K71 = -p.kr;
    K72 = -p.b2*p.kr;
    K73 = -p.a2*p.kr;
    K74 = 0;
    K75 = 0;
    K76 = 0;
    K77 = p.kr + p.ktr;
    k = [K11 K12 K13 K14 K15 K16 K17;
         K21 K22 K23 K24 K25 K26 K27;
         K31 K32 K33 K34 K35 K36 K37;
         K41 K42 K43 K44 K45 K46 K47
         K51 K52 K53 K54 K55 K56 K57;
         K61 K62 K63 K64 K65 K66 K67;
         K71 K72 K73 K74 K75 K76 K77];
    
    % Input disturbance matrix
    % TODO : Revise to actual model; add Force Function
    F = [0; 0; 0; 0*p.ktf; 0*p.ktf; 0*p.ktr; 0*p.ktr];
    
    % Pull out position and velocity, calculate acceleration.
    pos = [x(1); x(2); x(3); x(4); x(5); x(6); x(7)];
    vel = [x(8); x(9); x(10); x(11); x(12); x(13); x(14)];
    accel = m \ (F - b*vel - k*pos);
    
    % Package the data in the output
    xdot = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    xdot(1) = vel(1);
    xdot(2) = vel(2);
    xdot(3) = vel(3);
    xdot(4) = vel(4);
    xdot(5) = vel(5);
    xdot(6) = vel(6);
    xdot(7) = vel(7);
    xdot(8) = accel(1);
    xdot(9) = accel(2);
    xdot(10) = accel(3);
    xdot(11) = accel(4);
    xdot(12) = accel(5);
    xdot(13) = accel(6);
    xdot(14) = accel(7);
end
