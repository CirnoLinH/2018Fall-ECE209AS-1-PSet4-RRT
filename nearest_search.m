function [new_p,near_p,near_p_ind,vector_dir] = nearest_search(rrt_step,rand_p,vertices)

dist_rand = pdist2(vertices,rand_p);

[dist_min,near_p_ind]=min(dist_rand);

near_p=vertices(near_p_ind,:);

vector_dir = (rand_p - near_p)./dist_min;

if dist_min > rrt_step
    new_p = floor(near_p+rrt_step*vector_dir);
else
    new_p = rand_p;
end

end

