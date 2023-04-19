% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        q_start -> 1x4 vector denoting the start configuration
%        q_goal -> 1x4 vector denoting the goal configuration
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: path -> Nx4 matrix containing a collision-free path between
%                 q_start and q_goal, if a path is found. The first row
%                 should be q_start, the final row should be q_goal.
%         path_found -> Boolean denoting whether a path was found
function [path, path_found] = potential_fields(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_centers, sphere_radii)

    % Set up parameters for potential fields
    alpha = 100000; % Repulsion scaling factor
    beta = 5; % Attraction scaling factor
    threshold = 5; % Threshold distance to goal
    max_iterations = 100000; % Maximum number of iterations
    
    % Initialize variables
    q_current = q_start;
    path_found = false;
    path = q_current;
    iteration = 1;
    
    while norm(q_current - q_goal) > threshold && iteration <= max_iterations
        % Calculate repulsive forces from obstacles
        repulsive_forces = zeros(1, 4);
        for i = 1:size(sphere_centers, 1)
            obstacle_center = sphere_centers(i, :);
            obstacle_radius = sphere_radii(i);
            distance_to_obstacle = norm(q_current - obstacle_center) - obstacle_radius;
            if distance_to_obstacle < 0
                distance_to_obstacle = 0;
            end
            direction_to_obstacle = (q_current - obstacle_center) / norm(q_current - obstacle_center);
            repulsive_force_magnitude = alpha / distance_to_obstacle^2;
            repulsive_forces = repulsive_forces + direction_to_obstacle * repulsive_force_magnitude;
        end
        
        % Calculate attractive forces to goal
        direction_to_goal = (q_goal - q_current) / norm(q_goal - q_current);
        attractive_force_magnitude = beta * norm(q_goal - q_current);
        attractive_force = direction_to_goal * attractive_force_magnitude;
        
        % Calculate resulting force and update configuration
        total_force = attractive_force + repulsive_forces;
        q_new = q_current + total_force;
        q_new = min(max(q_new, q_min), q_max);
        
        % Check for collision with robot or obstacles
        collision = check_collision(robot, q_new, link_radius, sphere_centers, sphere_radii);
        if ~collision
            q_current = q_new;
            path = [path; q_current];
        end
        
        % Update iteration count
        iteration = iteration + 1;
    end
    
    % Check if a path was found
    if norm(q_current - q_goal) <= threshold
        path_found = true;
        path = [path; q_goal];
    end
end
