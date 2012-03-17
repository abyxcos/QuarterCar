% Ground distrubance(s)
function y=disturbance_step(t)
    global bump_height;
    y=0;
    
    if(t>3.0)
        y=bump_height;
    end
end