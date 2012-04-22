% Ground distrubance(s)

% disturbance_step - Determines disturbance input to the system
% Parameters:
%     t - The current time
% Returns:
%     The disturbance input to the car
function y = disturbance_step(t)
    global bump_height;
    y = 0;
    
    if (t > 3.0)
        y = bump_height;
        %y = bump_height * cos(8*t);
    end
end
