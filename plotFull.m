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
    disturbance_front=zeros([1,length(t)-1]);
    disturbance_back=zeros([1,length(t)-1]);
    y_p = [0; 0; 0; 0];
    for i=2:length(t)
        temp=modelFullInverse2(t(i), x(i,:), x(i-1,:), y_p, f_car);
        y(1, i) = temp(1);
        y(2, i) = temp(2);
        y(3, i) = temp(3);
        y(4, i) = temp(4);
        y_p = temp;
        disturbance_front(i)=disturbance_step(t(i));
        disturbance_back(i)=disturbance_step(t(i)-f_car.back_lag);
    end
    
    % Calculate the error between the input and the inverse
    error=zeros([4, length(t)-1]);
    for i=1:length(t)
        error(1, i)=y(1, i)-disturbance_step(t(i));
        error(2, i)=y(2, i)-disturbance_step(t(i));
        error(3, i)=y(3, i)-disturbance_step(t(i)-f_car.back_lag);
        error(4, i)=y(4, i)-disturbance_step(t(i)-f_car.back_lag);
    end
    
    % Generate the plots
    figure
    % Position row
    subplot(3,5,1);
    plot(t,x(:,1));
    title('x_s (offset of body)');
    ylabel('Meters');
    xlabel('Time (seconds)');

    subplot(3,5,2);
    plot(t,x(:,2));
    title('\theta (tilt of roll)');
    ylabel('Radians');
    xlabel('Time (seconds)');

    subplot(3,5,3);
    plot(t,x(:,2));
    title('\theta (tilt of pitch)');
    ylabel('Radians');
    xlabel('Time (seconds)');
    
    subplot(3,5,4);
    plot(t,x(:,4), t,x(:,5),'r');
    title('x_1 and x_2 (offset of front tires)');
    ylabel('Meters');
    xlabel('Time (seconds)');

    subplot(3,5,5);
    plot(t,x(:,6), t,x(:,7),'r');
    title('x_3 and x_4 (offset of back tires)');
    ylabel('Meters');
    xlabel('Time (seconds)');

    % Velocity row
    subplot(3,5,6);
    plot(t,x(:,8));
    title('v_s (velocity of body)');
    ylabel('Meters/second');
    xlabel('Time (seconds)');

    subplot(3,5,7);
    plot(t,x(:,9));
    title('\omega (velocity of roll)');
    ylabel('Radians/second');
    xlabel('Time (seconds)');

    subplot(3,5,8);
    plot(t,x(:,10));
    title('\theta (velocity of pitch)');
    ylabel('Radians/second');
    xlabel('Time (seconds)');
    
    subplot(3,5,9);
    plot(t,x(:,11), t,x(:,12),'r');
    title('v_1 and v_2 (velocity of front tires)');
    ylabel('Meters/second');
    xlabel('Time (seconds)');

    subplot(3,5,10);
    plot(t,x(:,13), t,x(:,14),'r');
    title('v_3 and v_4 (velocity of back tires)');
    ylabel('Meters/second');
    xlabel('Time (seconds)');
    
    % Acceleration row
    subplot(3,5,11);
    plot(t(1:end-1),F1);
    title('a_s (acceleration of body)');
    ylabel('Meters/second^2');
    xlabel('Time (seconds)');

    subplot(3,5,12);
    plot(t(1:end-1),F2);
    title('\alpha (acceleration of roll)');
    ylabel('Radians/second^2');
    xlabel('Time (seconds)');

    subplot(3,5,13);
    plot(t(1:end-1),F3);
    title('\alpha (acceleration of pitch)');
    ylabel('Radians/second^2');
    xlabel('Time (seconds)');
    
    subplot(3,5,14);
    plot(t(1:end-1),F4, t(1:end-1),F5,'r');
    title('a_1 and a_2 (acceleration of front tires)');
    ylabel('Meters/second^2');
    xlabel('Time (seconds)');
    
    subplot(3,5,15);
    plot(t(1:end-1),F6, t(1:end-1),F7,'r');
    title('a_3 and a_4 (acceleration of back tires)');
    ylabel('Meters/second^2');
    xlabel('Time (seconds)');

    figure
    hold on
    subplot(2,2,1);
    plot(t,y(1, :),'b', t,disturbance_front,'r', t,y(2, :),'b', t,disturbance_front,'r');
    title('Input Disturbance (y_1 and y_2)');
    legend('Calculated Disturbance (left)', 'Actual Disturbance (left)', 'Calculated Disturbance (right)', 'Actual Disturbance (right)', 'Location', 'NorthOutside');
    xlabel('Time (seconds)');
    ylabel('Meters');
        
    subplot(2,2,2);
    plot(t,y(3, :),'b', t,disturbance_back,'r', t,y(4, :),'b', t,disturbance_back,'r');
    title('Input Disturbance (y_3 and y_4)');
    legend('Calculated Disturbance (left)', 'Actual Disturbance (left)', 'Calculated Disturbance (right)', 'Actual Disturbance (right)', 'Location', 'NorthOutside');
    xlabel('Time (seconds)');
    ylabel('Meters');
    
    subplot(2,2,3);
    plot(t,error(1, :),t,error(2, :),'r');
    title('Estimation Error for front wheels');
    ylabel('Meters');
    xlabel('Time (seconds)');
    legend('y_1', 'y_2', 'Location', 'NorthOutside');
    
    subplot(2,2,4);
    plot(t,error(3, :),t,error(4, :),'r');
    title('Estimation Error for rear wheels');
    ylabel('Meters');
    xlabel('Time (seconds)');
    legend('y_3', 'y_4', 'Location', 'NorthOutside');
end