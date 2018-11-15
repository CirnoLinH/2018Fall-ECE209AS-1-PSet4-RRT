function path = path_search(edges)

% The path here is represented by the indices of the vertices.
% In this function, the path is inversely searched.

temp = edges(end,2);
path = edges(end,:);

while(1)
    ind= find(edges(:,1)==temp);
    temp_e = edges(ind,:);
    temp = temp_e(2);
    path = [path,temp];
    if temp == 1
        break;
    end
end

end
