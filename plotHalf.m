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

    % Run the inverse quarter car model
    y=zeros([2, length(t)-1]);
    disturbance=zeros([1,length(t)-1]);
    for i=1:length(t)-1
        temp=modelHalfInverse(t(i), x(i,:), [F1(i) F2(i) F3(i) F4(i)], h_car);
        y(1, i) = temp(1);
        y(2, i) = temp(2);
        disturbance(i)=disturbance_step(t(i));
    end
    
    % Generate the plots
    figure
    subplot(2,4,1);
    plot(t,x(:,1));
    xlabel('x_s (offset of body)');

    subplot(2,4,2);
    plot(t,x(:,2));
    xlabel('\theta (tilt of roll)');

    subplot(2,4,3);
    plot(t,x(:,3));
    xlabel('x_1 (offset of left tire)');

    subplot(2,4,4);
    plot(t,x(:,4));
    xlabel('x_2 (offset of right tire)');
    
    subplot(2,4,5);
    plot(t,x(:,5));
    xlabel('x_s (velocity of body)');

    subplot(2,4,6);
    plot(t,x(:,6));
    xlabel('\omega (velocity of roll)');

    subplot(2,4,7);
    plot(t,x(:,7));
    xlabel('v_1 (velocity of left tire)');

    subplot(2,4,8);
    plot(t,x(:,8));
    xlabel('v_2 (velocity of right tire)');
    
    figure
    hold on
    plot(t(1:end-1),y(1, :));
    plot(t(1:end-1),disturbance,'r');
    xlabel('input disturbance (from inverse dynamics)');
    ylabel('height (meters)');
end