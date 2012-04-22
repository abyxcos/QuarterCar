% Quarter Car Model Inverse 2

% modelQuarterInverse2 - Determines the disturbance input based off the
%     state
% Parameters:
%     t - The current time
%     x - The current position and velocity
%     x_p - The previous position and velocity
%     y_p - The previous input disturbance
%     p - The model parameters
% Returns:
%     The disturbance input to the car
function y = modelQuarterInverse2(t, x, x_p, y_p, p)

    global g;

    % Pull out position and velocity
    pos = [x(1); x(2)];
    vel = [x(3); x(4)];

    % Input disturbance matrix for previous state
    F = [-p.ms*g;
        p.ku*y_p];

    % Calculate the acceleration from the previous state
    pos_p = [x_p(1); x_p(2)];
    vel_p = [x_p(3); x_p(4)];
    accel = p.m \ (F - p.b*vel_p - p.k*pos_p);

    % Calculate the disturbance input matrix
    F = p.m*accel + p.b*vel + p.k*pos;

    % Pull out the input
    y = F(2) / p.ku;
end
