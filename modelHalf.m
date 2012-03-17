% Half Car Model

% modelHalf - Determines the new xdot values for the half car
% Parameters:
%     t - The current time
%     x - The current state of the model
%     p - The model parameters
% Returns:
%     The xdot for the model in its current state
function xdot = modelHalf(t, x, p)

    % Input disturbance matrix
    % TODO : Different steps per wheel???
    F = [-p.ms*p.grav; 0; p.kt*disturbance_step(t); p.kt*disturbance_step(t)];

    % Pull out position and velocity, calculate acceleration.
    pos = [x(1); x(2); x(3); x(4)];
    vel = [x(5); x(6); x(7); x(8)];
    accel = p.m \ (F - p.b*vel - p.k*pos);

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
