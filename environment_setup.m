function envir_m = environment_setup(~)

% Groundtruth environment setup


envir_m = zeros(3500,4500);

% Obstacle
for i = 1:1:3500
    for j = 1:1:4500
        if (i>=501 && i<=1000)||(i>=1501 && i<=2000)||(i>=2501 && i<=3000)
            if (j>=501 && j<=2000)
                envir_m(i,j) = 1;
            elseif (j>=2501 && j<=4000)
                envir_m(i,j) = 1;
            end
        end
        if (j<=15 || j>=4486 || i<=15 || i>=3486)
            envir_m(i,j) = 1;
        end
    end
end

% Target & Initial point
for i = 501:1:1000
    for j = 3001:1:3150
        envir_m(i,j) = 0;
    end
end
for i = 3486:1:3500
    for j = 2100:1:2300
        envir_m(i,j) = 0;
    end
end

end