% Generate the appropriate plots for the quarter car model
function plotQuarter(t, x)
    % Set up enviornment for the quarter car model
    init_globals;
    init_globals_quarter;
    
    % Run the quarter car model
    [t, x] = ode45(@(t, x) modelQuarter(t, x, q_car), [0 5], [0; 0; 0; 0]);

    % Determine the accelerations based off the returned velocities
    F1 = diff(x(:, 3))./diff(t);
    F2 = diff(x(:, 4))./diff(t);
    tt = 0:(t(end) / (length(F1) - 1)):t(end);
    
    % Run the inverse quarter car model
    %y=modelQuarterInverse(t, x, q_car);
    
    % Generate the plots
    subplot(1, 6, 1);
    plot(t, x(:, 1));
    xlabel('x_s (offset of body)');

    subplot(1, 6, 2);
    plot(t, x(:, 2));
    xlabel('x_u (offset of tire)');

    subplot(1, 6, 3);
    plot(t, x(:, 3));
    xlabel('v_s (velocity of body)');

    subplot(1, 6, 4);
    plot(t, x(:, 4));
    xlabel('v_u (velocity of tire)');

    subplot(1, 6, 5);
    plot(tt, F1);
    xlabel('a_s (acceleration of body)');

    subplot(1, 6, 6);
    plot(tt, F2);
    xlabel('a_u (acceleration of tire)');
end