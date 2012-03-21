% Generate the appropriate plots for the quarter car model
function plotHalf(t, x)
    close all;
    
    % Set up enviornment for the quarter car model
    init_globals;
    init_globals_half;
    
    % Run the quarter car model
    [t,x]=ode45(@(t,x) modelHalf(t, x, h_car), [0 5], [0; 0; 0; 0; 0; 0; 0; 0]);
    
    % Determine the accelerations based off the returned velocities
    F1 = diff(x(:, 5))./diff(t);
    F2 = diff(x(:, 6))./diff(t);
    F3 = diff(x(:, 7))./diff(t);
    F4 = diff(x(:, 8))./diff(t);
    tt = 0:(t(end) / (length(F1) - 1)):t(end);

    % Run the inverse half car model
    y=zeros([2, length(t)-1]);
    disturbance=zeros([1,length(t)-1]);
    y_p = [0; 0];
    for i=2:length(t)
        %temp=modelHalfInverse(t(i), x(i,:), [F1(i) F2(i) F3(i) F4(i)], h_car);
        temp=modelHalfInverse2(t(i), x(i,:), x(i-1,:), y_p, h_car);
        y(1, i) = temp(1);
        y(2, i) = temp(2);
        y_p = temp;
        disturbance(i)=disturbance_step(t(i));
    end
    
    % Calculate the error between the input and the inverse
    error=zeros([2, length(t)-1]);
    for i=1:length(t)
        error(1, i)=y(1, i)-disturbance_step(t(i));
        error(2, i)=y(2, i)-disturbance_step(t(i));
    end
    
    % Generate the plots
    figure
    subplot(2,4,1);
    plot(t,x(:,1));
    title('x_s (offset of body)');
    ylabel('Meters');
    xlabel('Time (seconds)');

    subplot(2,4,2);
    plot(t,x(:,2));
    title('\theta (tilt of roll)');
    ylabel('Radians');
    xlabel('Time (seconds)');

    subplot(2,4,3);
    plot(t,x(:,3));
    title('x_1 (offset of left tire)');
    ylabel('Meters');
    xlabel('Time (seconds)');

    subplot(2,4,4);
    plot(t,x(:,4));
    title('x_2 (offset of right tire)');
    ylabel('Meters');
    xlabel('Time (seconds)');
    
    subplot(2,4,5);
    plot(t,x(:,5));
    title('x_s (velocity of body)');
    ylabel('Meters/Second');
    xlabel('Time (seconds)');

    subplot(2,4,6);
    plot(t,x(:,6));
    title('\omega (velocity of roll)');
    ylabel('Radians/Second');
    xlabel('Time (seconds)');

    subplot(2,4,7);
    plot(t,x(:,7));
    title('v_1 (velocity of left tire)');
    ylabel('Meters/Second');
    xlabel('Time (seconds)');

    subplot(2,4,8);
    plot(t,x(:,8));
    title('v_2 (velocity of right tire)');
    ylabel('Meters/Second');
    xlabel('Time (seconds)');
    
    figure
    hold on
    subplot(1,4,1);
    plot(t,y(1, :), t,disturbance,'r');
    title('Input Disturbance (y_1)');
    legend('Calculated Disturbance', 'Actual Disturbance');
    xlabel('Time (seconds)');
    ylabel('Meters');
    
    subplot(1,4,2);
    plot(t,y(2, :), t,disturbance,'r');
    title('Input Disturbance (y_2)');
    legend('Calculated Disturbance', 'Actual Disturbance');
    xlabel('Time (seconds)');
    ylabel('Meters');
    
    subplot(1,4,3);
    plot(t,error(1, :));
    title('Estimation Error for y_2');
    ylabel('Meters');
    xlabel('Time (seconds)');

    subplot(1,4,4);
    plot(t,error(2, :));
    title('Estimation Error for y_2');
    ylabel('Meters');
    xlabel('Time (seconds)');
end