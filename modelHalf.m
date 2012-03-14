% Half Car Model

% modelHalf - Determines the new xdot values for the half car
% Parameters:
%     t - The current time
%     x - The current state of the model
%     p - The model parameters
% Returns:
%     The xdot for the model in its current state
function xdot = modelHalf(t, x, p)

    % Mass matrix
    m = [p.ms 0 0 0;
        0 p.Ix 0 0;
        0 0 p.m1 0;
        0 0 0 p.m2];
    
    % Damper matrix
    B11 = 2*p.bu;
    B12 = p.bu*p.b1 - p.bu*p.b2;
    B13 = -p.bu;
    B14 = -p.bu;
    B21 = p.bu*p.b1 - p.bu*p.b2;
    B22 = p.bu*p.b1^2 + p.bu*p.b2^2;
    B23 = -p.bu*p.b1;
    B24 = p.bu*p.b2;
    B31 = -p.bu;
    B32 = -p.bu*p.b1;
    B33 = p.bu;
    B34 = 0;
    B41 = -p.bu;
    B42 = p.bu*p.b2;
    B43 = 0;
    B44 = p.bu;
    b = [B11 B12 B13 B14;
        B21 B22 B23 B24;
        B31 B32 B33 B34;
        B41 B42 B43 B44];
    
    % Spring matrix
    K11 = 2*p.ku;
    K12 = p.ku*p.b1 - p.ku*p.b2;
    K13 = -p.ku;
    K14 = -p.ku;
    K21 = p.ku*p.b1 - p.ku*p.b2;
    K22 = p.ku*p.b1^2 + p.ku*p.b2^2;
    K23 = -p.ku*p.b1;
    K24 = p.ku*p.b2;
    K31 = -p.ku;
    K32 = -p.ku*p.b1;
    K33 = p.ku + p.kt;
    K34 = 0;
    K41 = -p.ku;
    K42 = p.ku*p.b2;
    K43 = 0;
    K44 = p.ku + p.kt;
    k = [K11 K12 K13 K14;
        K21 K22 K23 K24;
        K31 K32 K33 K34;
        K41 K42 K43 K44];
    
    % Input disturbance matrix
    % TODO : Revise to actual model; add Force Function
    F = [-p.ms*p.grav; 0; p.kt*disturbance_step(t); p.kt*disturbance_step(t)];
    
    % Pull out position and velocity, calculate acceleration.
    pos = [x(1); x(2); x(3); x(4)];
    vel = [x(5); x(6); x(7); x(8)];
    accel = m \ (F - b*vel - k*pos);
    
    % Package the data in the output
    xdot = [0; 0; 0; 0; 0; 0; 0; 0];
    xdot(1) = vel(1);
    xdot(2) = vel(2);
    xdot(3) = vel(3);
    xdot(4) = vel(4);
    xdot(5) = accel(1);
    xdot(6) = accel(2);
    xdot(7) = accel(3);
    xdot(8) = accel(4);
end
