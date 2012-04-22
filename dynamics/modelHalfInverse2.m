% Half Car Model Inverse

% modelHalfInverse2 - Determines the disturbance input based off the state
% Parameters:
%     t - The current time
%     x - The current position and velocity
%     x_p - The previous position and velocity
%     y_p - The previous input disturbance
%     p - The model parameters
% Returns:
%     The disturbance input to the car
function y = modelHalfInverse2(t, x, x_p, y_p, p)

    global g;

    % Pull out position and velocity
    pos = [x(1); x(2); x(3); x(4)];
    vel = [x(5); x(6); x(7); x(8)];

    % Input disturbance matrix for previous state
    F = [-p.ms*g; 0; p.kt*y_p(1); p.kt*y_p(2)];
    
    % Calculate the acceleration from the previous state
    pos_p = [x_p(1); x_p(2); x_p(3); x_p(4)];
    vel_p = [x_p(5); x_p(6); x_p(7); x_p(8)];
    accel = p.m \ (F - p.b*vel_p - p.k*pos_p);

    % Calculate the disturbance input matrix
    F = p.m*accel + p.b*vel + p.k*pos;

    % Pull out the input
    y = [0; 0];
    y(1) = F(3) / p.kt;
    y(2) = F(4) / p.kt;
end
