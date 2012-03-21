% Generate the appropriate plots for the quarter car model
function plotFull(t, x)
    close all;
    
    % Set up enviornment for the quarter car model
    init_globals;
    init_globals_full;
    
    % Run the quarter car model
    [t,x]=ode45(@(t,x) modelFull(t, x, f_car), [0 5], [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0]);
    
    % Determine the accelerations based off the returned velocities
    F1 = diff(x(:, 8))./diff(t);
    F2 = diff(x(:, 9))./diff(t);
    F3 = diff(x(:, 10))./diff(t);
    F4 = diff(x(:, 11))./diff(t);
    F5 = diff(x(:, 12))./diff(t);
    F6 = diff(x(:, 13))./diff(t);
    F7 = diff(x(:, 14))./diff(t);
    tt = 0:(t(end) / (length(F1) - 1)):t(end);

    % Run the inverse half car model
    y=zeros([4, length(t)-1]);
    disturbance=zeros([1,length(t)-1]);
    y_p = [0; 0; 0; 0];
    for i=2:length(t)
        temp=modelFullInverse2(t(i), x(i,:), x(i-1,:), y_p, f_car);
        y(1, i) = temp(1);
        y(2, i) = temp(2);
        y(3, i) = temp(3);
        y(4, i) = temp(4);
        y_p = temp;
        disturbance(i)=disturbance_step(t(i));
    end
    
    % Calculate the error between the input and the inverse
    error=zeros([4, length(t)-1]);
    for i=1:length(t)
        error(1, i)=y(1, i)-disturbance_step(t(i));
        error(2, i)=y(2, i)-disturbance_step(t(i));
        error(3, i)=y(3, i)-disturbance_step(t(i));
        error(4, i)=y(4, i)-disturbance_step(t(i));
    end
    
    % Generate the plots
    figure
    % Position row
    subplot(3,7,1);
    plot(t,x(:,1));
    title('x_s (offset of body)');
    ylabel('Meters');
    xlabel('Time (seconds)');

    subplot(3,7,2);
    plot(t,x(:,2));
    title('\theta (tilt of roll)');
    ylabel('Radians');
    xlabel('Time (seconds)');

    subplot(3,7,3);
    plot(t,x(:,2));
    title('\theta (tilt of pitch)');
    ylabel('Radians');
    xlabel('Time (seconds)');
    
    subplot(3,7,4);
    plot(t,x(:,4));
    title('x_1 (offset of left front tire)');
    ylabel('Meters');
    xlabel('Time (seconds)');

    subplot(3,7,5);
    plot(t,x(:,5));
    title('x_2 (offset of right front tire)');
    ylabel('Meters');
    xlabel('Time (seconds)');
    
    subplot(3,7,6);
    plot(t,x(:,6));
    title('x_3 (offset of left back tire)');
    ylabel('Meters');
    xlabel('Time (seconds)');

    subplot(3,7,7);
    plot(t,x(:,7));
    title('x_4 (offset of right back tire)');
    ylabel('Meters');
    xlabel('Time (seconds)');
    
    % Velocity row
    subplot(3,7,8);
    plot(t,x(:,8));
    title('v_s (velocity of body)');
    ylabel('Meters/second');
    xlabel('Time (seconds)');

    subplot(3,7,9);
    plot(t,x(:,9));
    title('\omega (velocity of roll)');
    ylabel('Radians/second');
    xlabel('Time (seconds)');

    subplot(3,7,10);
    plot(t,x(:,10));
    title('\theta (velocity of pitch)');
    ylabel('Radians/second');
    xlabel('Time (seconds)');
    
    subplot(3,7,11);
    plot(t,x(:,11));
    title('v_1 (velocity of left front tire)');
    ylabel('Meters/second');
    xlabel('Time (seconds)');

    subplot(3,7,12);
    plot(t,x(:,12));
    title('v_2 (velocity of right front tire)');
    ylabel('Meters/second');
    xlabel('Time (seconds)');
    
    subplot(3,7,13);
    plot(t,x(:,13));
    title('v_3 (velocity of left back tire)');
    ylabel('Meters/second');
    xlabel('Time (seconds)');

    subplot(3,7,14);
    plot(t,x(:,14));
    title('v_4 (velocity of right back tire)');
    ylabel('Meters/second');
    xlabel('Time (seconds)');
    
    % Acceleration row
    subplot(3,7,15);
    plot(t(1:end-1),F1);
    title('a_s (acceleration of body)');
    ylabel('Meters/second^2');
    xlabel('Time (seconds)');

    subplot(3,7,16);
    plot(t(1:end-1),F2);
    title('\alpha (acceleration of roll)');
    ylabel('Radians/second^2');
    xlabel('Time (seconds)');

    subplot(3,7,17);
    plot(t(1:end-1),F3);
    title('\alpha (acceleration of pitch)');
    ylabel('Radians/second^2');
    xlabel('Time (seconds)');
    
    subplot(3,7,18);
    plot(t(1:end-1),F4);
    title('a_1 (acceleration of left front tire)');
    ylabel('Meters/second^2');
    xlabel('Time (seconds)');

    subplot(3,7,19);
    plot(t(1:end-1),F5);
    title('a_2 (acceleration of right front tire)');
    ylabel('Meters/second^2');
    xlabel('Time (seconds)');
    
    subplot(3,7,20);
    plot(t(1:end-1),F6);
    title('a_3 (acceleration of left back tire)');
    ylabel('Meters/second^2');
    xlabel('Time (seconds)');

    subplot(3,7,21);
    plot(t(1:end-1),F7);
    title('a_4 (acceleration of right back tire)');
    ylabel('Meters/second^2');
    xlabel('Time (seconds)');

    figure
    hold on
    subplot(2,4,1);
    plot(t,y(1, :), t,disturbance,'r');
    title('Input Disturbance (y_1)');
    legend('Calculated Disturbance', 'Actual Disturbance');
    xlabel('Time (seconds)');
    ylabel('Meters');
    
    subplot(2,4,2);
    plot(t,y(2, :), t,disturbance,'r');
    title('Input Disturbance (y_2)');
    legend('Calculated Disturbance', 'Actual Disturbance');
    xlabel('Time (seconds)');
    ylabel('Meters');
    
    subplot(2,4,3);
    plot(t,y(3, :), t,disturbance,'r');
    title('Input Disturbance (y_3)');
    legend('Calculated Disturbance', 'Actual Disturbance');
    xlabel('Time (seconds)');
    ylabel('Meters');
    
    subplot(2,4,4);
    plot(t,y(4, :), t,disturbance,'r');
    title('Input Disturbance (y_4)');
    legend('Calculated Disturbance', 'Actual Disturbance');
    xlabel('Time (seconds)');
    ylabel('Meters');
    
    subplot(2,4,5);
    plot(t,error(1, :));
    title('Estimation Error for y_2');
    ylabel('Meters');
    xlabel('Time (seconds)');

    subplot(2,4,6);
    plot(t,error(2, :));
    title('Estimation Error for y_2');
    ylabel('Meters');
    xlabel('Time (seconds)');
    
    subplot(2,4,7);
    plot(t,error(3, :));
    title('Estimation Error for y_2');
    ylabel('Meters');
    xlabel('Time (seconds)');

    subplot(2,4,8);
    plot(t,error(4, :));
    title('Estimation Error for y_2');
    ylabel('Meters');
    xlabel('Time (seconds)');
end