% Full Car Model Inverse

% modelFullInverse2 - Determines the disturbance input based off the state
% Parameters:
%     t - The current time
%     x - The current position and velocity
%     x_p - The previous position and velocity
%     y_p - The previous input disturbance
%     p - The model parameters
% Returns:
%     The disturbance input to the car
function y = modelFullInverse2(t, x, x_p, y_p, p)

    global g;

    % Pull out position and velocity
    pos = [x(1); x(2); x(3); x(4); x(5); x(6); x(7)];
    vel = [x(8); x(9); x(10); x(11); x(12); x(13); x(14)];

    % Input disturbance matrix for previous state
    F = [-p.mu*p.grav;
        0;
        0;
        y_p(1)*p.ktf;
        y_p(2)*p.ktf;
        y_p(3)*p.ktr;
        y_p(4)*p.ktr];
    
    % Calculate the acceleration from the previous state
    pos_p = [x_p(1); x_p(2); x_p(3); x_p(4); x_p(5); x_p(6); x_p(7)];
    vel_p = [x_p(8); x_p(9); x_p(10); x_p(11); x_p(12); x_p(13); x_p(14)];
    accel = p.m \ (F - p.b*vel_p - p.k*pos_p);

    % Calculate the disturbance input matrix
    F = p.m*accel + p.b*vel + p.k*pos;

    % Pull out the input
    y = [0; 0; 0; 0];
    y(1) = F(4) / p.ktf;
    y(2) = F(5) / p.ktf;
    y(3) = F(6) / p.ktr;
    y(4) = F(7) / p.ktr;
end
