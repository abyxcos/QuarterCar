% Generate the appropriate plots for the quarter car model
function plotQuarter(t, x)
    subplot(2,4,1);
    plot(t,x(:,1));
    xlabel('x_s (offset of body)');

    subplot(2,4,2);
    plot(t,x(:,2));
    xlabel('x_u (offset of tire)');

    subplot(2,4,3);
    plot(t,x(:,3));
    xlabel('v_s (velocity of body)');

    subplot(2,4,4);
    plot(t,x(:,4));
    xlabel('v_u (velocity of tire)');
end