function [delta_y, yaw] = turnLeft(t, p, yaw)
    global delta_t2 t_avoidance2 y_pos t_old;

    if (t < (t_avoidance2 - delta_t2)) || (delta_t2 == 0)
        delta_y = 0;
        yaw = 0;
        return;
    end
    
    if y_pos(end) < (-p.b1-p.b2)
        turning_speed = pi/20;
    else
        turning_speed = -pi/20;
    end
    y_pos(end+1) = y_pos(end) + p.speed*(t-t_old) * sin(turning_speed);
    delta_y = p.speed * sin(turning_speed);
    yaw = turning_speed;
    t_old = t;
end