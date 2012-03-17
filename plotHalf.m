% Generate the appropriate plots for the quarter car model
function plotHalf(t, x)
    % Set up enviornment for the quarter car model
    init_globals;
    init_globals_half;
    % Run the quarter car model
    [t,x]=ode45(@(t,x) modelHalf(t, x, h_car), [0 5], [0; 0; 0; 0; 0; 0; 0; 0]);
    
    % Generate the plots
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
end