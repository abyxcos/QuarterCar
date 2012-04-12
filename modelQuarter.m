% Quarter Car Model

% modelQuarter - Determines the new xdot values for the quarter car
% Parameters:
%     t - The current time
%     x - The current state of the model
%     p - The model parameters
% Returns:
%     The xdot for the model in its current state
function xdot = modelQuarter(t, x, p)
    global g;
    
    % Input disturbance matrix
    F = [-p.ms*g;
        p.ku*disturbance_step(t)];

    % Pull out position and velocity, calculate acceleration.
    pos = [x(1); x(2)];
    vel = [x(3); x(4)];
    accel = p.m \ (F - p.b*vel - p.k*pos); % '\'==fast inverse
    
    % Package the data in the output
    xdot = [0; 0; 0; 0];
    xdot(1) = vel(1);
    xdot(2) = vel(2);
    xdot(3) = accel(1);
    xdot(4) = accel(2);
end
