function path_smooth = smooth_func(path,vertices,envir_set)

% This function is for path smoothing.
% The smoothing procedure starts from the first point on the path.


path_smooth = path(end);
temp_p = vertices(1,:);

while(1)
    length_path = length(path);
    for i=1:length_path
        % According to the point index in variable path, obtain the possible path.
        vec_path = vertices(path(i),:) - temp_p;
        
        % Normalize
        vec_dir = vec_path/norm(vec_path);
        no_collision_res = addtovertice_eval(envir_set,vertices(path(i),:),temp_p,vec_dir,60);
        
        % If no collision happens, we could add this point to the final smooth path.
        if no_collision_res
            path_smooth = [path_smooth, path(i)];
            temp_p = vertices(path(i),: );
            break;
        else
            continue;
        end
    end
    
    % The final part of the path
    vec_goal = vertices(end,:) - temp_p;
    % Normalize
    goal_dir = vec_goal/norm(vec_goal);
    or_goal = addtovertice_eval(envir_set,vertices(end,:),temp_p,goal_dir,60);
    
    % If the final part of the path has no collision, we could finish the loop.
    if or_goal
        path_smooth = [path_smooth, path(1)];
        break;
    else
        idx = find(path==path(i));
        path = path(1:idx);
    end
end

end



