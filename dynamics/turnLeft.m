function yaw = turnLeft(t, p, yaw)
    global delta_t2 t_avoidance2 y_pos t_old;
    
    if (t < (t_avoidance2 - delta_t2))
        delta_y = 0;
        yaw = 0;
        return;
    end
    
    if y_pos < (-p.b1-p.b2)
        % Yes, so return to yaw = 0
        turning_speed = p.turning_angle;
    else
        turning_speed = -p.turning_angle;
    end
    y_pos = y_pos + p.speed*(t-t_old) * sin(turning_speed);
    yaw = turning_speed;
    t_old = t;
end