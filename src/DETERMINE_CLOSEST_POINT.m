function [x, y] = DETERMINE_CLOSEST_POINT(roadMap, robotX, robotY)
% DETERMINE_CLOSEST_POINT  Nearest roadmap waypoint to the robot.
%
%   Returns the [x, y] coordinates of the roadmap waypoint closest to the
%   robot's current position, using straight-line (Euclidean) distance.
%   Used by PATH_PLANNER to decide where the robot should enter the roadmap.
%
%   Inputs
%     roadMap        : Nx2 matrix of candidate waypoints [x y]
%     robotX, robotY : current robot position [m]
%
%   Outputs
%     x, y : coordinates of the nearest waypoint [m]
%
%   See also PATH_PLANNER.

    minDistance = 5;          % larger than the 5 m arena diagonal bound
    resultIndex = 0;

    for i = 1:size(roadMap, 1)
        distance = sqrt((roadMap(i, 1) - robotX)^2 + (roadMap(i, 2) - robotY)^2);
        if distance < minDistance
            minDistance = distance;
            resultIndex = i;
        end
    end

    x = roadMap(resultIndex, 1);
    y = roadMap(resultIndex, 2);
end
