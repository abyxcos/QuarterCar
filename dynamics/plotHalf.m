% Generate the appropriate plots for the quarter car model
function plotHalf

close all;
    
    % Set up enviornment for the quarter car model
    initGlobals;
    initGlobalsHalf;
    
    % Run the quarter car model
    [t, x] = ode45(@(t, x) modelHalf(t, x, h_car), [0 sim_time], ...
        [0; 0; 0; 0; 0; 0; 0; 0]);
    
    % Determine the accelerations based off the returned velocities
    accel_b = diff(x(:, 5))./diff(t);
    accel_r = diff(x(:, 6))./diff(t);
    accel_t1 = diff(x(:, 7))./diff(t);
    accel_t2 = diff(x(:, 8))./diff(t);
    tt = 0 : (t(end) / (length(accel_b) - 1)) : t(end);

    % Run the inverse full car model to calculate the estimated input
    % disturbance
    y = zeros([2, length(t)-1]);
    disturbance = zeros([1, length(t)-1]);
    y_p = [0; 0];
    for i = 2 : length(t)
        temp=modelHalfInverse2(t(i), x(i, :), x(i-1, :), y_p, h_car);
        y(1, i) = temp(1);
        y(2, i) = temp(2);
        y_p = temp;
        disturbance(i) = disturbance_step(t(i));
    end
    
    % Calculate the error between the input and the inverse
    error = zeros([2, length(t)-1]);
    for i = 1 : length(t)
        error(1, i) = y(1, i) - disturbance_step(t(i));
        error(2, i) = y(2, i) - disturbance_step(t(i));
    end
    
    % Generate the plots
    figure
    
    % Body Offset
    subplot(2, 2, 1);
    plot(t, x(:, 1));
    title('x_s (Body Offset)');
    xlabel('Time (Seconds)');
    ylabel('Offset (Meters)');

    % Roll Tilt
    subplot(2, 2, 2);
    plot(t, x(:, 2));
    title('\theta (Roll Tilt)');
    xlabel('Time (Seconds)');
    ylabel('Tilt (Radians)');

    % Left Tire Offset
    subplot(2, 2, 3);
    plot(t, x(:, 3));
    title('x_1 (Left Tire Offset)');
    xlabel('Time (Seconds)');
    ylabel('Offset (Meters)');

    % Right Tire Offset
    subplot(2, 2, 4);
    plot(t, x(:, 4));
    title('x_2 (Right Tire Offset)');
    xlabel('Time (Seconds)');
    ylabel('Offset (Meters)');
    
    figure
    
    % Body Velocity
    subplot(2, 2, 1);
    plot(t, x(:, 5));
    title('x_s (Body Velocity)');
    xlabel('Time (Seconds)');
    ylabel('Velocity (Meters/Second)');

    % Roll Velocity
    subplot(2, 2, 2);
    plot(t, x(:, 6));
    title('\omega (Roll Velocity)');
    xlabel('Time (Seconds)');
    ylabel('Velocity (Radians/Second)');

    % Left Tire Velocity
    subplot(2, 2, 3);
    plot(t, x(:, 7));
    title('v_1 (Left Tire Velocity)');
    xlabel('Time (Seconds)');
    ylabel('Velocity (Meters/Second)');

    % Right Tire Velocity
    subplot(2, 2, 4);
    plot(t, x(:, 8));
    title('v_2 (Right Tire Velocity)');
    xlabel('Time (Seconds)');
    ylabel('Velocity (Meters/Second)');
    
    figure
    
    % Body Acceleration
    subplot(2, 2, 1);
    plot(t(1 : end-1), accel_b);
    title('a_s (Body Acceleration)');
    xlabel('Time (Seconds)');
    ylabel('Acceleration (Meters/Second^2)');
    
    % Roll Acceleration
    subplot(2, 2, 2);
    plot(t(1 : end-1), accel_r);
    title('\alpha (Roll Acceleration)');
    xlabel('Time (Seconds)');
    ylabel('Acceleration (Radians/Second^2)');
    
    % Left Tire Acceleration
    subplot(2, 2, 3);
    plot(t(1 : end-1), accel_t1);
    title('a_1 (Left Tire Acceleration)');
    xlabel('Time (Seconds)');
    ylabel('Acceleration (Meters/Second^2)');
    
    % Right Tire Acceleration
    subplot(2, 2, 4);
    plot(t(1 : end-1), accel_t2);
    title('a_2 (Right Tire Acceleration)');
    xlabel('Time (Seconds)');
    ylabel('Acceleration (Meters/Second^2)');
    
    figure
    
    % Disturbance Input (Left)
    subplot(2, 2, 1);
    plot(t, y(1, :), t, disturbance, 'r');
    title('Input Disturbance (Left Wheel)');
    legend('Estimated Disturbance', 'Actual Disturbance', ...
        'Location', 'NorthOutside');
    xlabel('Time (Seconds)');
    ylabel('Offset (Meters)');
    
    % Disturbance Input (Right)
    subplot(2, 2, 2);
    plot(t, y(2, :), t, disturbance,'r');
    title('Input Disturbance (Right Wheel)');
    legend('Estimated Disturbance', 'Actual Disturbance', ...
        'Location', 'NorthOutside');
    xlabel('Time (Seconds)');
    ylabel('Offset (Meters)');
    
    % Error in Calculated Disturbance (Left)
    subplot(2, 2, 3);
    plot(t, error(1, :));
    title('Estimation Error for Left Wheel');
    xlabel('Time (Seconds)');
    ylabel('Error (Meters)');

    % Error in Calculated Disturbance (Right)
    subplot(2, 2, 4);
    plot(t, error(2, :));
    title('Estimation Error for Right Wheel');
    xlabel('Time (Seconds)');
    ylabel('Error (Meters)');
end
