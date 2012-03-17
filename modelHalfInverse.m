% Half Car Model Inverse

% modelHalfInverse - Determines the disturbance input based off the state
% Parameters:
%     t - The current time
%     x - The current state of the model
%     p - The model parameters
% Returns:
%     The disturbance input to the car
function y = modelHalfInverse(t, x, p)

    % Input disturbance matrix
    % TODO : Different steps per wheel???
    F = [-p.ms*p.grav; 0; p.kt*disturbance_step(t); p.kt*disturbance_step(t)];

    % Pull out position and velocity, calculate acceleration.
    pos = [x(1); x(2); x(3); x(4)];
    vel = [x(5); x(6); x(7); x(8)];
    accel = [x(9); x(10); x(11); x(12)];

     % Calculate the disturbance input matrix
    F = p.m*accel + p.b*vel + p.k*pos;

    % Pull out the input
    y = [0; 0];
    y(1) = F(3) / p.kt;
    y(2) = F(4) / p.kt;
end
