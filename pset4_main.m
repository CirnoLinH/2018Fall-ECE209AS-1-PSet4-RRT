%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%  ECE 209AS-1 Problem Set 4
%  Hanren Lin
%  Chungyu Chen
%  University of California, Los Angeles

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% MAIN FUNCTION (Environment with obstacles)

clear
clc

% ======================================
% Initial Setup for the robot
%
%  To simplify the simulation, we would estimate the robot into a circle with a diameter of 114 mm. 
% (To make the R/2 be an integer.)


init_position = [3400,2200];
init_angle = 1/2*pi;
goal_position = [600,3080];
goal_angle = 1/2*pi;

R = 114/2;

% r: radius of the wheels
r = 20;
% ======================================
% The obstacle distribution plot
%
% environment_setup(): Constructing an environment with obstacle-crowded
%                      feature such as a parking lot.
% environment_setup_obstacle_free(): Constructing an obstacle-free environment.
%                                   (Except boundary for better demonstration.)
% pseudo_environment_setup(): Constructing an environment with dilation rate of R/2.
%                             All collision analysis could be conducted in the pseudo environment, and we could only
%                             evaluate if the center point of the robot would intersect with any obstacle in it.


% envir_m = environment_setup_obstacle_free();
envir_m = environment_setup();
pseudo_envir_m = pseudo_environment_setup(R);
[map_height,map_width] = size(envir_m);
% obstacle_set = obs_setup();


% ======================================
% RRT Algorithm
%

% Plot the environment only
colormap=[1 1 1;0 0 0;1 0 0;0 1 0;0 0 1];
imshow(uint8(envir_m),colormap)
hold on

% Clarify the position of the initial point and goal point
plot(init_position(2),init_position(1),'*b')
plot(goal_position(2),goal_position(1),'*b')

edges = [];

% rrt_step decides the step of RRT tree, i.e. the sample rate of RRT tree.


rrt_step = 50;
vertices_set = init_position;

heading_angle = init_angle;

% Decided by the rrt_step
p=0.3;


% The maximum time of RRT iteration was set to 6000.
% If RRT tree doesn't reach the goal position within 6000 iterations, no result would be returned.
tic
for k = 1:6000
    
    % Decide if the RRT tree reaches the goal position or not
    arrival_eval_res=arrival_eval(vertices_set,goal_position,rrt_step/4,heading_angle,goal_angle);
    if arrival_eval_res
        vertices_set=[vertices_set;goal_position];
        edges = [edges;[size(vertices_set,1),size(vertices_set,1)-1]];
        break;
    end
    
    % Generate random sample points
    if rand <= p
        rand_position = goal_position;
    else
        rand_position = [randi(map_height),randi(map_width)];
    end
    
    
    % Collision Judgement
    
%     if envir_m(rand_position(1,1),rand_position(1,2)) == 1
    if pseudo_envir_m(rand_position(1,1),rand_position(1,2)) == 1
%         fprintf('I was here');
        continue;
    end
    
    % Get nearest point
    [new_p,near_p,near_p_ind,vector_dir] = nearest_search(rrt_step,rand_position,vertices_set);
    
%     heading_angle = atan2(rand_position(1) - near_p(1), rand_position(2) - near_p(2));
    
    % Decide whether adding new point or not
    addtovertice_res = addtovertice_eval(pseudo_envir_m,new_p,near_p,vector_dir,10);
%   addtovertice_res = addtovertice_eval(envir_m,new_p,near_p,vector_dir,10);

    if addtovertice_res
        vertices_set=[vertices_set;new_p];
        temp = size(vertices_set,1);
        edges = [edges;[temp,near_p_ind]];
    else
        continue;
    end
    
    % Real-time plotting of RRT tree expanding
    plot([near_p(1,2),new_p(1,2)],[near_p(1,1),new_p(1,1)],'-b');
    drawnow
end


if k == 6000
    fprintf('Error - RRT Tree could not find a possible path. Please try again.\n')
else
    % Path Search
    path = path_search(edges);
    % Path smoothing
    path_smooth = smooth_func(path,vertices_set,pseudo_envir_m);
%     path_smooth = [1,path(2)];
    % Plot original path
    plot(vertices_set(path,2),vertices_set(path,1),'-r')
    % Plot smoothed path
    plot(vertices_set(path_smooth,2),vertices_set(path_smooth,1),'-g');
    Frame(k+1) = getframe(gcf);
    
    
    % Angle velocity input calculation
    seg_num = size(path_smooth,2) - 1;
    if seg_num == 1
        dt = 1/seg_num;
    else
        % (Assuming one wheel has w = 0, time for taking turn is 0.07s)
        dt = (1-0.07*(seg_num-1))/seg_num;
    end
    for i = 1:seg_num
        dist = pdist2(vertices_set(path_smooth(1,i),:),vertices_set(path_smooth(1,i+1),:));
        % Since the size of the environment is relatively large, we ignore the angular velocity limit here.
        w(i) = dist/(r*dt);
    end
    
    % Angular Velocity when taking turns (Not calculated here)
end
toc

