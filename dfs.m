function [path, path_found] = dfs(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
 % Initialize path and path_found
    path = [];
    path_found = false;

    % Create a stack as a cell array
    stack = {};
    stack{end + 1} = q_start;
    visited = containers.Map(mat2str(q_start), true);

    % DFS loop
    while ~isempty(stack)
        % Pop the top node from the stack
        current_config = stack{end};
        stack(end) = [];

        % Check if current config is q_goal
        if isequal(current_config, q_goal)
            path = current_config;
            path_found = true;
            return;
        end

        % Generate child configs
        child_configs = generate_child_nodes(robot, current_config, q_min, q_max);
        for i = 1:numel(child_configs)
            child_config = child_configs(:, i);

            % Check if child config is within joint limits
            if ~is_within_joint_limits(child_config, q_min, q_max)
                continue;
            end

            % Check for collision at child config
            if check_collision(robot, child_config, link_radius, sphere_centers, sphere_radii)
                continue;
            end

            % Check if child config has been visited
            if visited.isKey(mat2str(child_config))
                continue;
            end

            % Add child config to the stack and mark as visited
            stack{end + 1} = child_config;
            visited(mat2str(child_config)) = true;
        end
    end

    % If no path is found
    disp("Error: No valid path found.");
end

function result = is_within_joint_limits(q, q_min, q_max)
    % Check if q is within joint limits
    result = all(q >= q_min) && all(q <= q_max);
end

function result = check_collision(robot, q, link_radius, sphere_centers, sphere_radii)
    % Check for collision at the given configuration q
    % Implementation of collision checking algorithm
    result = false;  % Replace with actual collision checking logic
end

function child_configs = generate_child_nodes(robot, q, q_min, q_max)
    % Generate child nodes from the current configuration q
    % Implementation of generating child nodes
    child_configs = [];  % Replace with actual implementation
end

