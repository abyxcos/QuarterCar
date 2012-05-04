function [delta_y, yaw] = turnLeft(t, p)
    global delta_t2;
    y_pos = 0;

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
    turning_speed = 0;
    delta_y = p.speed * sin(turning_speed);
    yaw = turning_speed;
end