% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        samples -> num_samples x 4 matrix, vertices in the roadmap
%        adjacency -> num_samples x num_samples matrix, the weighted
%                     adjacency matrix denoting edges in the roadmap
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

function [path, path_found] = PRM(robot, samples, adjacency, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
    nqstart = optimalneighbor(q_start, robot, samples,link_radius, sphere_centers, sphere_radii);
    nqgoal = optimalneighbor(q_goal, robot, samples,link_radius, sphere_centers, sphere_radii);
    
    path_steps = shortestpath(graph(adjacency), nqstart, nqgoal);
    path = [q_start; samples(path_steps,:) ; q_goal];
    
    if approvepath(robot, path, link_radius, sphere_centers, sphere_radii)
        path_found = true;
    else
        path = [];
        path_found = false;
    end
end

function collifree = approvepath(robot, path, link_radius, sphere_centers, sphere_radii)
    collifree = true;
    
   
    for q = path
        if check_collision(robot, q, link_radius, sphere_centers, sphere_radii)
            collifree = false;
            break;
        end
    end
    

    if collifree
        for j = 1:size(path, 1) - 1
            q_1 = path(j,:);
            q_2 = path(j+1,:);
            if check_edge(robot, q_1, q_2, link_radius, sphere_centers, sphere_radii)
                collifree = false;
                break;
            end
        end
    end
end


function [neighbor_index, neighbor] = optimalneighbor(q, robot, samples, link_radius, sphere_centers, sphere_radii)
    ranking = ranknodesbydistance(q, samples);
    
    for i = 1:size(ranking, 1) 
        neighbor_index = ranking(i);
        neighbor = samples(neighbor_index);
        if ~check_edge(robot, q, neighbor, link_radius, sphere_centers, sphere_radii)
            break;
        end
    end
end


function ranking = ranknodesbydistance(z, points)
    distances = vecnorm(points - z, 2, 2);
    [~, I] = sort(distances);
    ranking = I(2:end,:);
end

