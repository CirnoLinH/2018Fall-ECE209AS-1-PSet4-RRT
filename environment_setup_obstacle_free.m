function envir_m = environment_setup_obstacle_free()
% Groundtruth environment setup


envir_m = zeros(3500,4500);

% Obstacle
for i = 1:1:3500
    for j = 1:1:4500
        if (j<=15 || j>=4486 || i<=15 || i>=3486)
            envir_m(i,j) = 1;
        end
    end
end


for i = 501:1:1000
    for j = 2901:1:3150
        envir_m(i,j) = 0;
    end
end

end