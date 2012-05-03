% Ground surface

% make_some_ground - Makes some ground for use in the animation
% Parameters:
%     t_start - The starting time
%     t_end   - The ending time
% Returns:
%     The 3d ground matrix
%function [x,y,z] = make_some_ground(t_start, t_end)
function z = make_some_ground(t_start, t_end)
    j=1;
    for i=t_start:0.1:t_end
        ground(j)=disturbance_step(i);
        j=j+1;
    end
    
    x=zeros(length(ground));
    y=zeros(length(ground));
    z=zeros(length(ground));
    for i=1:length(ground)
        z(i,:)=ground;
    end
end