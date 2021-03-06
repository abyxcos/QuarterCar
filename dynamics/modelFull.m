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
    global y_pos delta_t2 t_avoidance t_old;
    delta_y = 0;
    yaw = 0;
    missed_bump = 1;

    % Are we far enough away yet?
    % Bump centered at y=0, car width wide
    if (t < 2.99)
        [delta_y, yaw] = turnLeft(t, p, x(16));
    end
    %if y_pos(end) < (-p.b1-p.b2)
    if x(15) < (-p.b1-p.b2)
        missed_bump = 0;
    end
    
    % Input disturbance matrix
    % Due to car length, the back wheels hit later
    F = [-p.mu*g;
        0;
        0;
        missed_bump*disturbance_step(t)*p.ktf;
        missed_bump*disturbance_step(t)*p.ktf;
        missed_bump*disturbance_step(t-p.back_lag)*p.ktr;
        missed_bump*disturbance_step(t-p.back_lag)*p.ktr;
    ];
    
    % Pull out position and velocity, calculate acceleration.
    pos = [x(1); x(2); x(3); x(4); x(5); x(6); x(7)];
    vel = [x(8); x(9); x(10); x(11); x(12); x(13); x(14)];
    accel = p.m \ (F - p.b*vel - p.k*pos);
    
    % Package the data in the output
    %xdot = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    xdot = zeros(16,1);
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
    
    xdot(15) = p.speed*sin(yaw);
    xdot(16) = yaw;
end
