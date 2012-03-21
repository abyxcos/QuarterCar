% Generate the appropriate plots for the quarter car model
function plotQuarter(t, x)
    close all;
    
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
    y=zeros([1,length(t)-1]);
    disturbance=zeros([1,length(t)-1]);
    y_p = 0;
    for i=2:length(t)
        %y(i)=modelQuarterInverse(t(i), x(i,:), [F1(i) F2(i)], q_car);
        y(i)=modelQuarterInverse2(t(i), x(i,:), x(i-1,:), y_p, q_car);
        y_p = y(i);
        disturbance(i)=disturbance_step(t(i));
    end
    
    % Calculate the error between the input and the inverse
    error=zeros([1, length(t)]);
    for i=1:length(t)-1
        error(i)=y(i)-disturbance_step(t(i));
    end
    
    % Generate the plots
    figure
    subplot(1, 3, 1);
    plot(t, x(:, 1), t, x(:, 2), 'r');
    xlabel('time (seconds)');
    ylabel('offset (meters)');
    %title('x_s (offset of body)');
    legend('x_s (body offset)', 'x_u (tire offset)', 'Location', 'NorthOutside');

    subplot(1, 3, 2);
    plot(t, x(:, 3), t, x(:, 4), 'r');
    xlabel('time (seconds)');
    ylabel('velocity (meters/second)');
    %title('v_s (velocity of body)');
    legend('v_s (body velocity)', 'v_u (tire velocity)', 'Location', 'NorthOutside');

    subplot(1, 3, 3);
    plot(t(1:end-1), F1, t(1:end-1), F2, 'r');
    xlabel('time (seconds)');
    ylabel('acceleration (meters/second^2)');
    %title('a_s (acceleration of body)');
    legend('a_s (body acceleration)', 'a_u (tire acceleration)', 'Location', 'NorthOutside');
    
    figure
    hold on
    subplot(1, 2, 1);
    plot(t,y,t,disturbance,'r');
    xlabel('time (seconds)');
    ylabel('offset (meters)');
    title('input disturbance (from inverse dynamics)');
    legend('Calculated Disturbance', 'Actual Disturbance');
    
    subplot(1, 2, 2);
    plot(t,error);
    xlabel('time (seconds)');
    ylabel('offset (meters)');
    title('estimation error');
end