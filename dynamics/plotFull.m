% Generate the appropriate plots for the quarter car model
function plotFull

    close all;
    
    % Set up enviornment for the quarter car model
    initGlobals;
    initGlobalsFull;
    
    % Run the quarter car model
    [t, x] = ode45(@(t, x) modelFull(t, x, f_car), [0 5], ...
        [f_car.h_body; 0; 0; ...
         f_car.h_wheel; f_car.h_wheel; f_car.h_wheel; f_car.h_wheel; ...
         0; 0; 0; ...
         0; 0; 0; 0]);
    
    % Determine the accelerations based off the returned velocities
    accel_b = diff(x(:, 8))./diff(t);
    accel_r = diff(x(:, 9))./diff(t);
    accel_p = diff(x(:, 10))./diff(t);
    accel_t1 = diff(x(:, 11))./diff(t);
    accel_t2 = diff(x(:, 12))./diff(t);
    accel_t3 = diff(x(:, 13))./diff(t);
    accel_t4 = diff(x(:, 14))./diff(t);
    tt = 0 : (t(end) / (length(accel_b) - 1)) : t(end);

    % Run the inverse full car model to calculate the estimated input
    % disturbance
    y = zeros([4, length(t)-1]);
    disturbance_front = zeros([1, length(t)-1]);
    disturbance_back = zeros([1, length(t)-1]);
    y_p = [0; 0; 0; 0];
    for i = 2 : length(t)
        temp = modelFullInverse(t(i), x(i, :), x(i-1, :), y_p, f_car);
        y(1, i) = temp(1);
        y(2, i) = temp(2);
        y(3, i) = temp(3);
        y(4, i) = temp(4);
        y_p = temp;
        disturbance_front(i) = disturbance_step(t(i));
        disturbance_back(i) = disturbance_step(t(i) - f_car.back_lag);
    end
    
    % Calculate the error between the input and the inverse
    error = zeros([4, length(t)-1]);
    for i = 1 : length(t)
        error(1, i) = y(1, i) - disturbance_step(t(i));
        error(2, i) = y(2, i) - disturbance_step(t(i));
        error(3, i) = y(3, i) - disturbance_step(t(i) - f_car.back_lag);
        error(4, i) = y(4, i) - disturbance_step(t(i) - f_car.back_lag);
    end
    
    
    %
    % Generate the plots
    %
    figure

    % Body Offset
    subplot(3, 5, 1);
    plot(t, x(:, 1));
    title('x_s (Body Offset)');
    xlabel('Time (Seconds)');
    ylabel('Offset (Meters)');

    % Car Roll
    subplot(3, 5, 2);
    plot(t, x(:, 2));
    title('\theta (Roll Tilt)');
    xlabel('Time (Seconds)');
    ylabel('Tilt (Radians)');

    % Car Pitch
    subplot(3, 5, 3);
    plot(t, x(:, 3));
    title('\theta (Pitch Tilt)');
    xlabel('Time (Seconds)');
    ylabel('Tilt (Radians)');
    
    % Front Tires Offset
    subplot(3, 5, 4);
    plot(t, x(:, 4), t, x(:, 5), 'r');
    title('x_1 and x_2 (Front Tires Offset)');
    xlabel('Time (Seconds)');
    ylabel('Offset (Meters)');

    % Back Tires Offset
    subplot(3, 5, 5);
    plot(t, x(:, 6), t, x(:, 7), 'r');
    title('x_3 and x_4 (Back Tires Offset)');
    xlabel('Time (Seconds)');
    ylabel('Offset (Meters)');

    % Body Velocity
    subplot(3, 5, 6);
    plot(t, x(:, 8));
    title('v_s (Body Velocity)');
    xlabel('Time (Seconds)');
    ylabel('Velocity (Meters/Second)');

    % Roll Velocity
    subplot(3, 5, 7);
    plot(t, x(:, 9));
    title('\omega (Roll Velocity)');
    xlabel('Time (Seconds)');
    ylabel('Velocity (Radians/Second)');

    % Pitch Velocity
    subplot(3, 5, 8);
    plot(t, x(:, 10));
    title('\theta (Pitch Velocity)');
    xlabel('Time (Seconds)');
    ylabel('Velocity (Radians/Second)');
    
    % Front Tires Velocity
    subplot(3, 5, 9);
    plot(t, x(:, 11), t, x(:, 12), 'r');
    title('v_1 and v_2 (Front Tires Velocity)');
    xlabel('Time (Seconds)');
    ylabel('Velocity (Meters/Second)');

    % Back Tires Velocity
    subplot(3, 5, 10);
    plot(t, x(:, 13), t, x(:, 14), 'r');
    title('v_3 and v_4 (Back Tires Velocity)');
    xlabel('Time (Seconds)');
    ylabel('Velocity (Meters/Second)');
    
    % Body Acceleration
    subplot(3, 5, 11);
    plot(t(1 : end-1), accel_b);
    title('a_s (Body Acceleration)');
    xlabel('Time (Seconds)');
    ylabel('Acceleration (Meters/Second^2)');

    % Roll Acceleration
    subplot(3, 5, 12);
    plot(t(1 : end-1), accel_r);
    title('\alpha (Roll Acceleration)');
    xlabel('Time (Seconds)');
    ylabel('Acceleration (Radians/Second^2)');

    % Pitch Accleration
    subplot(3, 5, 13);
    plot(t(1 : end-1), accel_p);
    title('\alpha (Pitch Acceleration)');
    xlabel('Time (Seconds)');
    ylabel('Acceleration (Radians/Second^2)');
    
    % Front Tires Acceleration
    subplot(3, 5, 14);
    plot(t(1 : end-1), accel_t1, t(1 : end-1), accel_t2, 'r');
    title('a_1 and a_2 (Front Tires Acceleration)');
    xlabel('Time (Seconds)');
    ylabel('Acceleration (Meters/Second^2)');
    
    % Back Tires Acceleration
    subplot(3, 5, 15);
    plot(t(1 : end-1), accel_t3, t(1 : end-1), accel_t4, 'r');
    title('a_3 and a_4 (Back Tires Acceleration)');
    xlabel('Time (Seconds)');
    ylabel('Acceleration (Meters/Second^2)');

    figure
    
    % Input Disturbance (Front)
    subplot(2, 2, 1);
    plot(t, y(1, :), 'b', t, disturbance_front, 'r', t, y(2, :), 'b', ...
        t, disturbance_front, 'r');
    title('Input Disturbance (y_1 and y_2)');
    legend('Calculated Disturbance (Left)', ...
        'Actual Disturbance (Left)', 'Calculated Disturbance (Right)', ...
        'Actual Disturbance (Right)', 'Location', 'NorthOutside');
    xlabel('Time (Seconds)');
    ylabel('Offset (Meters)');

    % Input Disturbance (Back)
    subplot(2, 2, 2);
    plot(t, y(3, :), 'b', t, disturbance_back, 'r', t, y(4, :), 'b', ...
        t, disturbance_back, 'r');
    title('Input Disturbance (y_3 and y_4)');
    legend('Calculated Disturbance (Left)', ...
        'Actual Disturbance (Left)', 'Calculated Disturbance (Right)', ...
        'Actual Disturbance (Right)', 'Location', 'NorthOutside');
    xlabel('Time (Seconds)');
    ylabel('Offset (Meters)');
    
    % Error in Calculated Disturbance (Front)
    subplot(2, 2, 3);
    plot(t, error(1, :), t, error(2, :), 'r');
    title('Estimation Error For Front Wheels');
    ylabel('Erro (Meters)');
    xlabel('Time (Seconds)');
    legend('y_1', 'y_2', 'Location', 'NorthOutside');
    
    % Error in Calculated Disturbance (Back)
    subplot(2, 2, 4);
    plot(t, error(3, :), t, error(4, :), 'r');
    title('Estimation Error For Back Wheels');
    ylabel('Error (Meters)');
    xlabel('Time (Seconds)');
    legend('y_3', 'y_4', 'Location', 'NorthOutside');
    
    
    %
    % Make an animation of the car over time
    %
    figure;

    % Body face references
    body_face = [1 2 3 4;
        2 6 7 3;
        4 3 7 8;
        1 5 8 4;
        1 2 6 5;
        5 6 7 8];
    
    % Body and tire patch objects
    body = patch('FaceColor', 'r');
    t_br = patch('FaceColor', 'k');
    t_bl = patch('FaceColor', 'k');
    t_fr = patch('FaceColor', 'k');
    t_fl = patch('FaceColor', 'k');
    
    % Reference unit sphere for tires
    [t_ref_x, t_ref_y, t_ref_z] = sphere();
    
    % Set the camera angle
    view(45, 30);
    axis([-2 2 -2 2 -.3 .1]);
    
    % Make a video file (30 frames per second over 5 seconds)
    video = VideoWriter('car.avi');
    video.FrameRate = 30;
    open(video);

    frame_time = 0;
    for i = 1 : length(t)
        % Calculate the body position
        body_y_r = 0 - f_car.b2*cos(x(i, 2));
        body_y_l = 0 + f_car.b1*cos(x(i, 2));
        body_x_b = 0 - f_car.a2*cos(x(i, 3));
        body_x_f = 0 + f_car.a1*cos(x(i, 3));
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
        
        % Check if this frame should be captured and update the next
        % capture time
        if (t(i) >= frame_time)
            writeVideo(video, getframe);
            frame_time = frame_time + 1/30;
        end
    end

    close(video);
end
