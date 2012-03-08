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
    C15 = -p.cf
    C16 = -p.cr;
    C17 = -p.cr;
    C21 = C12;
    C22 = p.b1^2*p.cf + p.b2^2*p.cf + p.b1^1*p.cr+p.b2^2*p.cr;
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
    b = [C11 C12 C13 C14 C15 C16 C17;
        C21 C22 C23 C24 C25 C26 C27;
        C31 C32 C33 C34 C35 C36 C37;
        C41 C42 C43 C44 0 0 0;
        -p.cf p.b2*p.cf p.a1*p.cf 0 p.cf 0 0;
        -p.cr p.b1*p.cr -p.a2*p.cr 0 0 p.cr 0;
        -p.cr -p.b2*p.cr -p.a2*p.cr 0 0 0 p.cr];
    
    % Spring matrix
    k11 = 2*p.kf + 2*p.kr;
    k12 = p.b1*p.kf - p.b2*p.kf - p.b1*p.kr + p.b2*p.kr;
    k21 = k12;
    k13 = 2*p.a2*p.kr - 2*p.a1*p.kf;
    k31 = k13;
    k22 = p.kR + p.b1^2*p.kf + p.b2^2*p.kf + p.b1^2*p.kr + p.b2^2*p.kr;
    k23 = p.a1*p.b2*p.kf - p.a1*p.b1*p.kf - p.a2*p.b1*p.kr + p.a2*p.b2*p.kr;
    k32 = k23;
    k24 = -p.b1*p.kf - 1/p.w*p.kR;
    k42 = k24;
    k25 = p.b2*p.kf + 1/p.w*p.kR;
    k52 = k25;
    k33 = 2*p.kf*p.a1^2 + 2*p.kr*p.a2^2;
    k44 = p.kf + p.ktf + 1/p.w^2*p.kR;
    k55 = p.kf + p.ktf + 1/p.w^2*p.kR;
    k = [k11 k12 k13 -p.kf -p.kf -p.kr -p.kr;
         k21 k22 k23 k24 k25 p.b1*p.kr -p.b2*p.kr;
         k31 k32 k33 p.a1*p.kf p.a1*p.kf -p.a2*p.kr -p.a2*p.kr;
         -p.kf k42 p.a1*p.kf k44 -p.kR/p.w^2 0 0;
         -p.kf k52 p.a1*p.kf -p.kR/p.w^2 k55 0 0;
         -p.kr p.b1*p.kr -p.a2*p.kr 0 0 (p.kr+p.ktr) 0;
         -p.kr -p.b2*p.kr -p.a2*p.kr 0 0 0 (p.kr+p.ktr)];
    
    % Input disturbance matrix
    % TODO : Revise to actual model; add Force Function
    F = [0; 0; 0; 0*p.ktf; 0*p.ktf; 0*p.ktf; 0*p.ktf];
    
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
