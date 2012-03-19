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
    for i=1:length(t)-1
        y(i)=modelQuarterInverse(t(i), x(i,:), [F1(i) F2(i)], q_car);
        disturbance(i)=disturbance_step(t(i));
    end
    
    % Calculate the error between the input and the inverse
    error=zeros([1, length(t)-1]);
    for i=1:length(t)-1
        error(i)=y(i)-disturbance_step(t(i));
    end
    
    % Generate the plots
    figure
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
    plot(t(1:end-1), F1);
    xlabel('a_s (acceleration of body)');

    subplot(1, 6, 6);
    plot(t(1:end-1), F2);
    xlabel('a_u (acceleration of tire)');
    
    figure
    hold on
    subplot(1, 2, 1);
    plot(t(1:end-1),y,t(1:end-1),disturbance,'r');
    xlabel('input disturbance (from inverse dynamics)');
    ylabel('height (meters)');
    
    subplot(1, 2, 2);
    plot(t(1:end-1),error);
    ylabel('estimation error (meters)');
end