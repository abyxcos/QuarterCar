% Quarter Car Model Inverse

% modelQuarterInverse - Determines the disturbance input based off the state
% Parameters:
%     t - The current time
%     x - The current state of the model
%     p - The model parameters
% Returns:
%     The disturbance input to the car
function y = modelQuarter(t, x, p)

    % Pull out position, velocity, and acceleration.
    pos = [x(1); x(2)];
    vel = [x(3); x(4)];
    accel = [x(5); x(6)];

    % Calculate the disturbance input matrix
    F = p.m*accel + p.b*vel + p.k*pos;

    % Pull out the input
    y = F(2) / p.ku;
end
