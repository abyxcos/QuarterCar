% Quarter Car Model

% modelQuarter - Determines the new xdot values for the quarter car
% Parameters:
%     t - The current time
%     x - The current state of the model
%     p - The model parameters
% Returns:
%     The xdot for the model in its current state
function xdot = modelQuarter(t, x, p)

    % Mass matrix
    m = [p.ms 0;
        0 p.mu];
    
    % Damper matrix
    b = [p.bs -p.bs;
        -p.bs (p.bs+p.bu)];
    
    % Spring matrix
    k = [p.ks -p.ks;
        -p.ks (p.ks+p.ku)];
    
    % Input disturbance matrix
    % TODO : Revise to actual model; add Force Function
    F = [-p.ms*p.grav;
        p.ku*disturbance_step(t)];
    
    % Pull out position and velocity, calculate acceleration.
    pos = [x(1); x(2)];
    vel = [x(3); x(4)];
    accel = m \ (F - b*vel - k*pos); % '\'==fast inverse
    
    % Package the data in the output
    xdot = [0; 0; 0; 0];
    xdot(1) = vel(1);
    xdot(2) = vel(2);
    xdot(3) = accel(1);
    xdot(4) = accel(2);
end
