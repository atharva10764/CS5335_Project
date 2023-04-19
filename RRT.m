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

function [path, path_found] = RRT(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
    step_size = 0.05;
    q_goal_sample_rate = 0.08;
    max_iter = 1000;
    
    tree = [q_start, 0];
    
   
    for i = 1:max_iter
        
        if rand < q_goal_sample_rate
            q_rand = q_goal;
        else
            q_rand = q_min + rand(1, robot.n) .* (q_max - q_min);
        end
        
       
        distances = vecnorm(tree(:,1:robot.n) - q_rand, 2, 2);
        [~, nearest_vertex_idx] = min(distances);
        q_near = tree(nearest_vertex_idx, 1:robot.n);
        
        
        if distances(nearest_vertex_idx) > step_size
            q_new = q_near + (q_rand - q_near) / distances(nearest_vertex_idx) * step_size;
        else
            q_new = q_rand;
        end
        
        
        if check_collision(robot, q_new', link_radius, sphere_centers, sphere_radii)
            continue;
        end
        
      
        new_vertex_idx = size(tree, 1) + 1;
        tree(new_vertex_idx, :) = [q_new, nearest_vertex_idx];
        
     
        if vecnorm(q_new - q_goal, 2) < step_size
            goal_vertex_idx = size(tree, 1);
            break;
        end
    end
    
   
    if vecnorm(tree(end, 1:robot.n) - q_goal, 2) >= step_size
        path_found = false;
        path = [];
        return;
    else
        path_found = true;
    end
    
   
    path = [tree(end, 1:robot.n)];
    vertex_idx = goal_vertex_idx;
    while vertex_idx ~= 1
        path = [tree(vertex_idx, 1:robot.n); path];
        vertex_idx = tree(vertex_idx, robot.n + 1);
    end
    path = [q_start; path];
end