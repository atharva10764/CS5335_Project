% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        path -> Nx4 matrix containing a collision-free path between
%                q_start and q_goal
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: smoothed_path -> Nx4 matrix containing a smoothed version of the
%                          input path, where some unnecessary intermediate
%                          waypoints may have been removed

function smoothed_path = smooth(robot, path, link_radius, sphere_centers, sphere_radii)
    if nargin < 5
        resolution = 11;
    end
    smoothed_path = [path(1,:)];
    i = 1;
    while i < size(path, 1)
        j = i+1;
        while j < size(path, 1)
            if ~check_edge(robot, path(i,:), path(j,:), link_radius, sphere_centers, sphere_radii)
                j = j+1;
            else
                j = j-1;
                break;
            end
        end
        if j >= size(path, 1)
            j = size(path, 1);
        end
        smoothed_path = [smoothed_path; path(j,:)];
        i = j;
    end
end