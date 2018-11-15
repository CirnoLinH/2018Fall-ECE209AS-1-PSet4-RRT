function arrival_res = arrival_eval(vertices,goal_position,rrt_step,heading_angle,goal_angle)

dist = pdist2(vertices(end,:),goal_position);

if dist <= rrt_step/10 && (abs(heading_angle-goal_angle)<=(1/90*pi)||abs(heading_angle+goal_angle)<=(1/90*pi))
    arrival_res=1;
else
    arrival_res=0;
end

end