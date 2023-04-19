% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples in PRM
%        num_neighbors -> Integer denoting number of closest neighbors to
%                         consider in PRM
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: samples -> num_samples x 4 matrix, sampled configurations in the
%                    roadmap (vertices)
%         adjacency -> num_samples x num_samples matrix, the weighted
%                      adjacency matrix denoting edges between roadmap
%                      vertices. adjacency(i,j) == 0 if there is no edge
%                      between vertex i and j; otherwise, its value is the
%                      weight (distance) between the two vertices. For an
%                      undirected graph, the adjacency matrix should be
%                      symmetric: adjacency(i,j) == adjacency(j,i)

function [samples, adjacency] = roadmap(robot, q_min, q_max, num_samples, num_neighbors, link_radius, sphere_centers, sphere_radii)

samples = zeros(num_samples, robot.n);
    for i = 1:num_samples
        while true
            q = q_min + rand(1, robot.n) .* (q_max - q_min);
            if ~check_collision(robot, q', link_radius, sphere_centers, sphere_radii)
                samples(i,:) = q;
                break;
            end
        end
    end
     adjacency = zeros(num_samples);
     if nargin < 9
        resolution = 11;
    end
      % Create edges
    for i = 1:num_samples
        q1 = samples(i, :);
        distances = vecnorm(samples - q1, 2, 2);
        [~, neighbor_indices] = sort(distances);
        num_added = 0;
        for j = 1:num_samples
            if num_added >= num_neighbors
                break;
            end
            if i == neighbor_indices(j)
                continue;
            end
            q2 = samples(neighbor_indices(j), :);
            if isequal(adjacency(i, neighbor_indices(j)), adjacency(neighbor_indices(j), i))
                if check_collision(robot, [q1;q2], link_radius, sphere_centers, sphere_radii, resolution)
                    adjacency(i, neighbor_indices(j)) = 0;
                    adjacency(neighbor_indices(j), i) = 0;
                else
                    adjacency(i, neighbor_indices(j)) = distances(neighbor_indices(j));
                    adjacency(neighbor_indices(j), i) = distances(neighbor_indices(j));
                    num_added = num_added + 1;
                end
            else
                num_added = num_added + 1;
            end
        end
    end
end