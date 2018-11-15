function pseudo_envir_m = pseudo_environment_setup(R)

% Pseudo environment setup


pseudo_envir_m = zeros(3500,4500);

% Obstacle
for i = 1:1:3500
    for j = 1:1:4500
        if (i>=501-R && i<=1000+R)||(i>=1501-R && i<=2000+R)||(i>=2501-R && i<=3000+R)
            if (j>=501-R && j<=2000+R)
                pseudo_envir_m(i,j) = 1;
            elseif (j>=2501-R && j<=4000+R)
                pseudo_envir_m(i,j) = 1;
            end
        end
        if (j<=15+R || j>=4486-R || i<=15+R || i>=3486-R)
            pseudo_envir_m(i,j) = 1;
        end
    end
end

% Target & Initial point
for i = 501-R:1:1000+R
    for j = 3001+R:1:3150-R
        pseudo_envir_m(i,j) = 0;
    end
end
for i = 3486-R:1:3500
    for j = 2100+R:1:2300-R
        pseudo_envir_m(i,j) = 0;
    end
end

end