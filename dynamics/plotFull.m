% Generate the appropriate plots for the full car model
function [t, x, f_car, t_avoidance] = plotFull(delta_t, t_avoidance)
    %close all;
    global delta_t2 t_avoidance2 y_pos t_old;
    delta_t2=delta_t;
    t_avoidance2=t_avoidance;
    y_pos = 0;
    t_old = 0;
    
    % Set up enviornment for the full car model
    initGlobals;
    initGlobalsFull;
    
    % Run the full car model
    %[t, x] = ode45(@(t, x) modelFull(t, x, f_car), [0-delta_t sim_time], ...
    [t, x] = ode45(@(t, x) modelFull(t, x, f_car), (0-delta_t):0.1:sim_time, ...
        [f_car.h_body; 0; 0; ...
         f_car.h_wheel; f_car.h_wheel; f_car.h_wheel; f_car.h_wheel; ...
         0; 0; 0; ...       % Body position and roll
         0; 0; 0; 0; ...    % Wheel position
         0; 0;]);           % Y position and yaw
         %t_avoidance-delta_t; 0;]); % Pass through time to turn at
    
%     if delta_t ~= 0
%         figure
%         axis([0 10 -10 10]);
%         hold on;
%         size(t)
%         size(y_pos')
%         plot(t,x(:,15),'b');
%         plot(t,x(:,16),'r');
%         plot(y_pos','g');
%         plot(t,(-f_car.b1-f_car.b2),'y');
%         hold off;
%         figure
%         plot(y_pos','g');
%     end
        
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
        
        % See if the car in front of us just hit a bump
        % Only check this if we're the second in line (running on car at
        % delta_t==0, as we don't have a third car yet.
        if (delta_t == 0)
            if max(y_p-y(i-1)) > 0.015 % Maximum error before we avoid
                if t_avoidance == 0
                    t_avoidance = t(i);
                end
            end
        end
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
end
