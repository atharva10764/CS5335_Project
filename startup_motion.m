
% Required algorithm_Type -> ("PRM", "RRT,"Potential Field")


function [samples, adjacency] = startup_motion(algorithm_Type,samples, adjacency)

    close all;
    

    % Create robot
	robot = create_robot();

 % Start and goal configuration
q_start = [0 -pi/4 0 -pi/4];
q_goal = [0 -3 0 -3];

% Minimum and maximum joint angles for each joint
q_min = [-pi/2 -pi 0 -pi];
q_max = [pi/2 0 0 0];

% Radius of each robot link's cylindrical body
link_radius = 0.03;

 
    % Set up spherical obstacle
    sphere_center = [0.5 0 0];
    sphere_radius = 0.25;

    % Plot robot and obstacle
    robot.plot(q_start);
    hold on;	
    draw_sphere(sphere_center,sphere_radius)
    % Plot the robot in a given configuration
    q = q_start;
    robot.plot(q);
    % Check if configuration is within joint limits / in collision
    in_bounds = (all(q >= q_min) && all(q <= q_max));
    in_collision = check_collision(robot, q, link_radius, sphere_center, sphere_radius);
    if in_bounds
        disp('Robot configuration is within joint limits.');
    else
        disp('Robot configuration is not within joint limits.');
    end
    if in_collision
        disp('Robot configuration is in collision.');
    else
        disp('Robot configuration is not in collision.');
    end
  
   if algorithm_Type == "PRM" 
        r = 0.38;
        % Create more spherical obstacles
        sphere_centers = [sphere_center; 0 0.5 0; 0 -0.5 0];
        sphere_radii = [sphere_radius; r; r];
        for i = 2:size(sphere_centers, 1)
            draw_sphere(sphere_centers(i,:)', sphere_radii(i));
        end
    % Parameters for PRM
    num_samples = 1000;
    num_neighbors = 25;  
    tic;
    % If pre-computed roadmap is not provided,
    % compute the roadmap using roadmap function
    if nargin < 3
        [samples, adjacency] = roadmap(robot, q_min, q_max, num_samples, num_neighbors, link_radius, sphere_center, sphere_radius);
    end
    % Use the roadmap to find a path from q_start to q_goal
    % TODO: Implement this function
    [path, path_found] = PRM(robot, samples, adjacency, q_start, q_goal, link_radius, sphere_center, sphere_radius);
    % Visualize the trajectory, if one is found
    if path_found
        fprintf('Path found with %d intermediate waypoints:\n', size(path, 1) - 2);
        disp(path);
        waypoints = interpolate_path(path);
        total_distance = 0;
        for i = 1:size(waypoints, 1)-1
        p1 = waypoints(i, :);
        p2 = waypoints(i+1, :);
        distance = norm(p2 - p1);
        total_distance = total_distance + distance;
        end
        fprintf('Total distance traveled: %f units.\n', total_distance);
        robot.plot(waypoints, 'fps', 10);
     else
        disp('No path found.');
     end

    % Stop measuring elapsed time and display the result
        elapsed_time = toc;
        fprintf('Elapsed time: %f seconds.\n', elapsed_time);
    end
  if algorithm_Type == "RRT"
        tic;

      % Use the RRT algorithm to find a path from q_start to q_goal
      r = 0.38;
        % Create more spherical obstacles
        sphere_centers = [sphere_center; 0 0.5 0; 0 -0.5 0];
        sphere_radii = [sphere_radius; r; r];
        for i = 2:size(sphere_centers, 1)
            draw_sphere(sphere_centers(i,:)', sphere_radii(i));
        end
      [path, path_found] = RRT(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_center, sphere_radius);
    if path_found
        fprintf('Path found with %d intermediate waypoints:\n', size(path, 1) - 2);
        disp(path);
        waypoints = interpolate_path(path);
        total_distance = 0;
        for i = 1:size(waypoints, 1)-1
        p1 = waypoints(i, :);
        p2 = waypoints(i+1, :);
        distance = norm(p2 - p1);
        total_distance = total_distance + distance;
        end
        fprintf('Total distance traveled: %f units.\n', total_distance);
        robot.plot(waypoints, 'fps', 10);
     else
        disp('No path found.');
     end     % Stop measuring elapsed time and display the result
        elapsed_time = toc;
        fprintf('Elapsed time: %f seconds.\n', elapsed_time);
  end
  
    
    if algorithm_Type == "Potential_field"
         r = 0.38;
        % Create more spherical obstacles
        sphere_centers = [sphere_center; 0 0.5 0; 0 -0.5 0];
        sphere_radii = [sphere_radius; r; r];
        tic;
        for i = 2:size(sphere_centers, 1)
            draw_sphere(sphere_centers(i,:)', sphere_radii(i));
        end
          % Use the potential fields algorithm to find a path from q_start to q_goal
        [path, path_found] = potential_fields(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_center, sphere_radius);
        if path_found
            fprintf('Path found with %d intermediate waypoints:\n', size(path, 1) - 2);
            disp(path);
        % If trajectory is found, smooth the trajectory
            % TODO: Implement this function
            smoothed_path = smooth(robot, path, link_radius, sphere_center, sphere_radius);
            % Visualize the smoothed trajectory
            fprintf('Smoothed path found with %d intermediate waypoints:\n', size(smoothed_path, 1) - 2);
            disp(smoothed_path);
            robot.plot(interpolate_path(smoothed_path), 'fps', 10);
        else
            disp('No path found.');
        end
         % Stop measuring elapsed time and display the result
        elapsed_time = toc;
        fprintf('Elapsed time: %f seconds.\n', elapsed_time);
    end
% ========== Helper functions ==========

% Given a path consisting of configuration waypoints,
% interpolates the waypoints to give a less abrupt trajectory.
% Consecutive pairs of waypoints are interpolated along a straight line
% in configuration space, where the newly generated points are
% less than max_velocity apart in configuration space.
% This helper function is primarily used for visualization purposes.
function trajectory = interpolate_path(path, max_velocity)
    if nargin < 2
        max_velocity = 0.05;
    end
    trajectory = [path(1,:)];
    for i = 2:size(path, 1)
        vec = path(i,:) - path(i-1,:);
        num_ticks = ceil(norm(vec) / max_velocity);
        ticks = linspace(0, 1, num_ticks + 1)';
        segment = repmat(path(i-1,:), num_ticks, 1) + repmat(ticks(2:end,:), 1, length(path(i,:))) .* repmat(vec, num_ticks, 1);
        trajectory = [trajectory; segment];
    end
end

% Create a 4-DOF arm with 2 links
function robot = create_robot()
    L(1) = Link([0 0 0 1.571]);
    L(2) = Link([0 0 0 -1.571]);
    L(3) = Link([0 0.4318 0 -1.571]);
    L(4) = Link([0 0 0.4318 1.571]);    
    robot = SerialLink(L, 'name', 'robot');
end

% Draw a sphere with specified center position and radius
function draw_sphere(position, radius)
    [X,Y,Z] = sphere;
    X = X * radius;
    Y = Y * radius;
    Z = Z * radius;
    X = X + position(1);
    Y = Y + position(2);
    Z = Z + position(3);
    surf(X,Y,Z);
end
end