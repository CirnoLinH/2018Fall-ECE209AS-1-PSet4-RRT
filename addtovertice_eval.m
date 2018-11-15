function add_eval_res = addtovertice_eval(envir_set,new_p,near_p,vector_dir,insert_p_num)

dist_gap = norm(new_p - near_p)/insert_p_num;
temp =1:insert_p_num;
insert_point = repmat(near_p,insert_p_num,1) + temp'.*dist_gap*vector_dir;
insert_point =[floor(insert_point);new_p];

% Represent all points in terms of indices
insert_num = sub2ind(size(envir_set),insert_point(:,1),insert_point(:,2));
coll_set = find(envir_set(insert_num)==1,1);

% coll_set has any element: confirmed collision
% coll_set has no element: no collision
if ~isempty(coll_set)
    add_eval_res=0;
else
    add_eval_res=1;
end

end

