% Make an animation of the car over time
function animateFull(x, t, f_car)
    close all;
    figure;
    
    % Body face references
    body_face = [1 2 3 4;
                 2 6 7 3;
                 4 3 7 8;
                 1 5 8 4;
                 1 2 6 5;
                 5 6 7 8];
    
    % Body and tire patch objects
    ground_p = patch('FaceColor', 'g');
    body = patch('FaceColor', 'r');
    t_br = patch('FaceColor', 'k');
    t_bl = patch('FaceColor', 'k');
    t_fr = patch('FaceColor', 'k');
    t_fl = patch('FaceColor', 'k');
    
    % Reference unit sphere for tires
    [t_ref_x, t_ref_y, t_ref_z] = sphere();
    
    % Make the ground
    ground_z = make_some_ground(0, t(end));
    size(ground_z);
    
    % Set the camera angle
    %view(45, 30);
    view(0, 0);
    axis([0 300 -20 20 -.3 .1]);
    
    % Make a video file (30 frames per second over 5 seconds)
    video = VideoWriter('car.avi');
    video.FrameRate = 30;
    open(video);

    frame_time = 0;
    for i = 1 : length(t)
        % Calculate the body position
        body_y_r = 0 - f_car.b2*cos(x(i, 2));
        body_y_l = 0 + f_car.b1*cos(x(i, 2));
        body_x_b = t(i)*f_car.speed - f_car.a2*cos(x(i, 3));
        body_x_f = t(i)*f_car.speed + f_car.a1*cos(x(i, 3));
        body_z_br = x(i, 1) - f_car.b2*sin(x(i, 2)) - f_car.a2*sin(x(i, 3));
        body_z_bl = x(i, 1) + f_car.b2*sin(x(i, 2)) - f_car.a2*sin(x(i, 3));
        body_z_fr = x(i, 1) - f_car.b2*sin(x(i, 2)) + f_car.a1*sin(x(i, 3));
        body_z_fl = x(i, 1) + f_car.b2*sin(x(i, 2)) + f_car.a1*sin(x(i, 3));
        body_vert = [body_x_b body_y_r body_z_fr; % Bottom, back, right
            body_x_b body_y_l body_z_fl; % Bottom, back, left
            body_x_f body_y_l body_z_bl; % Bottom, front, left
            body_x_f body_y_r body_z_br; % Bottom, front, right
            body_x_b body_y_r (body_z_fr + .05); % Top, back, right
            body_x_b body_y_l (body_z_fl + .05); % Top, back, left
            body_x_f body_y_l (body_z_bl + .05); % Top, front, left
            body_x_f body_y_r (body_z_br + .05)]; % Top, front, right
        set(body, 'Faces', body_face, 'Vertices', body_vert);
        
        % Calculate the tire positions
        [t_face, t_vert, t_color] = ...
            surf2patch((t_ref_x/8) + body_x_b, ...
            (t_ref_y/8) + body_y_r, ...
            (t_ref_z/64) - 0.25 + x(i, 6));
        set(t_br, 'Faces', t_face, 'Vertices', t_vert);

        [t_face, t_vert, t_color] = ...
            surf2patch((t_ref_x/8) + body_x_b, ...
            (t_ref_y/8) + body_y_l, ...
            (t_ref_z/64) - 0.25 + x(i, 7));
        set(t_bl, 'Faces', t_face, 'Vertices', t_vert);

        [t_face, t_vert, t_color] = ...
            surf2patch((t_ref_x/8) + body_x_f, ...
            (t_ref_y/8) + body_y_r, ...
            (t_ref_z/64) - 0.25 + x(i, 5));
        set(t_fr, 'Faces', t_face, 'Vertices', t_vert);

        [t_face, t_vert, t_color] = ...
            surf2patch((t_ref_x/8) + body_x_f, ...
            (t_ref_y/8) + body_y_l, ...
            (t_ref_z/64) - 0.25 + x(i, 4));
        set(t_fl, 'Faces', t_face, 'Vertices', t_vert);
        
        [g_face, g_vert, g_color] = ...
            surf2patch(0:1.3:65, -20:0.8:20, ground_z-0.3);
        set(ground_p, 'Faces', g_face, 'Vertices', g_vert);
        
        % Check if this frame should be captured and update the next
        % capture time
        if (t(i) >= frame_time)
            writeVideo(video, getframe);
            frame_time = frame_time + 1/30;
        end
    end

    close(video);
end