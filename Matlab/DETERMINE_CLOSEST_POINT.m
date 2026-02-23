function [x, y] = DETERMINE_CLOSEST_POINT(roadMap, robotX, robotY)
% This function determines the closest point in the roadmap with respect to
% robot's current position.
% Returns x, y of the closest position. 
    minDistance = 5;
    resultIndex = 0;
    for i= 1:1:size(roadMap, 1)
        distance = sqrt((roadMap(i,1) - robotX)^2 +(roadMap(i,2) - robotY)^2);
        if distance < minDistance
            minDistance = distance;
            resultIndex = i;
        end
    end
    x = roadMap(resultIndex, 1);
    y = roadMap(resultIndex,2);
end
