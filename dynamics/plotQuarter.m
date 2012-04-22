% Generate the appropriate plots for the quarter car model
function plotQuarter()

    close all;
    
    % Set up enviornment for the quarter car model
    initGlobals;
    initGlobalsQuarter;
    
    % Run the quarter car model
    [t, x] = ode45(@(t, x) modelQuarter(t, x, q_car), [0 5], [0; 0; 0; 0]);

    % Determine the accelerations based off the returned velocities
    accel_b = diff(x(:, 3))./diff(t);
    accel_t = diff(x(:, 4))./diff(t);
    tt = 0 : (t(end) / (length(accel_b) - 1)) : t(end);

    % Run the inverse quarter car model to calculate the estimated input
    % disturbance
    y = zeros([1, length(t)-1]);
    disturbance = zeros([1, length(t)-1]);
    y_p = 0;
    for i = 2 : length(t)
        y(i) = modelQuarterInverse2(t(i), x(i, :), x(i-1, :), y_p, q_car);
        y_p = y(i);
        disturbance(i) = disturbance_step(t(i));
    end
    
    % Calculate the error between the input and the inverse
    error = zeros([1, length(t)]);
    for i = 1 : length(t)-1
        error(i) = y(i) - disturbance_step(t(i));
    end
    
    % Generate the plots
    figure
    
    % Body and Tire Offsets
    subplot(1, 3, 1);
    plot(t, x(:, 1), t, x(:, 2), 'r');
    xlabel('Time (Seconds)');
    ylabel('Offset (Meters)');
    legend('x_s (Body Offset)', 'x_u (Tire Offset)', ...
        'Location', 'NorthOutside');

    % Body and Tire Velocities
    subplot(1, 3, 2);
    plot(t, x(:, 3), t, x(:, 4), 'r');
    xlabel('Time (Seconds)');
    ylabel('Velocity (Meters/Second)');
    legend('v_s (Body Velocity)', 'v_u (Tire Velocity)', ...
        'Location', 'NorthOutside');

    % Body and Tire Accelerations
    subplot(1, 3, 3);
    plot(t(1 : end-1), accel_b, t(1 : end-1), accel_t, 'r');
    xlabel('Time (Seconds)');
    ylabel('Ccceleration (Meters/Second^2)');
    legend('a_s (Body Acceleration)', 'a_u (Tire Acceleration)', ...
        'Location', 'NorthOutside');
    
    figure
    
    % Input Disturbance
    subplot(1, 2, 1);
    plot(t, y, t, disturbance, 'r');
    xlabel('Time (Seconds)');
    ylabel('Offset (Meters)');
    title('Input Disturbance (From Inverse Dynamics)');
    legend('Calculated Disturbance', 'Actual Disturbance', ...
        'Location', 'NorthOutside');
    
    % Error in Calculated Disturbance
    subplot(1, 2, 2);
    plot(t, error);
    xlabel('Time (Seconds)');
    ylabel('Error (Meters)');
    title('Estimation Error');
end
