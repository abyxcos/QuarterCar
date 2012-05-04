% Full Car Model

% modelFull - Determines the new xdot values for the full car
% Parameters:
%     t - The current time
%     x - The current state of the model
%     p - The model parameters
% Returns:
%     The xdot for the model in its current state
function xdot = modelFull(t, x, p)

    global g;

    % Deal with turning
    y_pos = x(15);
    yaw = x(16);
    t_old = x(17);
    
    % Are we far enough away yet?
    % Bump centered at y=0, car width wide
    if y_pos > p.b1+p.b2
        % Yes, so return to yaw = 0
        if yaw > 0
            turning_speed = p.turning_speed;
            if turning_speed > p.turning_angle  % Conviently they line up
                turning_speed = p.turning_angle;
            end
            %turning_speed = turning_speed/(t-t_old);
        end
        turning_speed = 0;
    else
        turning_speed = p.turning_speed;
            if turning_speed > p.turning_angle  % Conviently they line up
                turning_speed = p.turning_angle;
            end
            turning_speed = -turning_speed;
            %turning_speed = -turning_speed/(t-t_old);
    end
    y_pos = y_pos + p.speed*(t-t_old) * sin(turning_speed);
    yaw = turning_speed/(t-t_old);
    
    % Input disturbance matrix
    % Due to car length, the back wheels hit later
    F = [-p.mu*g;
        0;
        0;
        disturbance_step(t)*p.ktf;
        disturbance_step(t)*p.ktf;
        disturbance_step(t-p.back_lag)*p.ktr;
        disturbance_step(t-p.back_lag)*p.ktr;
    ];
    
    % Pull out position and velocity, calculate acceleration.
    pos = [x(1); x(2); x(3); x(4); x(5); x(6); x(7)];
    vel = [x(8); x(9); x(10); x(11); x(12); x(13); x(14)];
    accel = p.m \ (F - p.b*vel - p.k*pos);
    
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
    xdot(15) = y_pos;
    xdot(16) = yaw;
    xdot(17) = x(17);   % Pass t_avoidance back through
    xdot(18) = t;
end
