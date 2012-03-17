% Generate the appropriate plots for the quarter car model
function plotQuarter(t, x)
    % Set up enviornment for the quarter car model
    init_globals;
    init_globals_quarter;
    
    % Run the quarter car model
    [t,x]=ode45(@(t,x) modelQuarter(t, x, q_car), [0 5], [0; 0; 0; 0]);
    
    % Run the inverse quarter car model
    %y=modelQuarterInverse(t, x, q_car);
    
    % Generate the plots
    subplot(1,4,1);
    plot(t,x(:,1));
    xlabel('x_s (offset of body)');

    subplot(1,4,2);
    plot(t,x(:,2));
    xlabel('x_u (offset of tire)');

    subplot(1,4,3);
    plot(t,x(:,3));
    xlabel('v_s (velocity of body)');

    subplot(1,4,4);
    plot(t,x(:,4));
    xlabel('v_u (velocity of tire)');
end