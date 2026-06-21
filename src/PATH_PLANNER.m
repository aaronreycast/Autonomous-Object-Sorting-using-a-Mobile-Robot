function [navigationMatrix, navigationPointer] = PATH_PLANNER(roadMapObject0, roadMapObject1, currentObject, objectColor, robotX, robotY, pppX, pppY)
% PATH_PLANNER  Build the navigation queue for the current object/color.
%
%   Given the active object slot (Object_0 = bottom, Object_1 = top), its
%   detected color and the robot pose, this function returns an ordered list
%   of waypoints (segments) that drive the robot to the correct "Pre-Plow
%   Position" (PPP) and then plows the object into its matching collection
%   zone in a single straight motion. The returned queue is consumed by the
%   reusable navigation routine in the Simulink state machine.
%
%   Inputs
%     roadMapObject0, roadMapObject1 : Nx2 roadmap waypoints per object slot
%     currentObject                  : 0 (bottom) or 1 (top)
%     objectColor                    : 1 = red, 2 = green, 3 = blue
%     robotX, robotY                 : current robot position [m]
%     pppX, pppY                     : pre-plow position for this object [m]
%
%   Outputs
%     navigationMatrix  : 3x2 ordered waypoints [x y] (unused rows are 0 0)
%     navigationPointer : index of the first active waypoint in the queue
%
%   See also DETERMINE_CLOSEST_POINT.

    % Collection zone center positions, one row per color [r; g; b]
    collectionZonePositionObject0 = [1.75 2.5; 1.9 -1.9; -2.25 0];
    collectionZonePositionObject1 = [2.2 2.2; 2.2 -2.2; -2.2 0];

    % Default queue (empty) and pointer.
    navigationPointer = 3;
    navigationMatrix = [0 0; 0 0; 0 0];

    if currentObject == 0

        [x, y] = DETERMINE_CLOSEST_POINT(roadMapObject0, robotX, robotY);
        if [x, y] == [pppX, pppY]
            % Already at the pre-plow position: queue only the plow motion.
            x = collectionZonePositionObject0(objectColor, 1);
            y = collectionZonePositionObject0(objectColor, 2);
            navigationMatrix = [0 0; 0 0; x y];
            return
        end

        % Plan a path to reach the pre-plow position (PPP).
        if objectColor == 1                 % red
            navigationPointer = navigationPointer - 1;
            navigationMatrix(navigationPointer, 1) = pppX;
            navigationMatrix(navigationPointer, 2) = pppY;
        elseif objectColor == 3             % blue
            % Add PPP to the queue.
            navigationPointer = navigationPointer - 1;
            navigationMatrix(navigationPointer, 1) = pppX;
            navigationMatrix(navigationPointer, 2) = pppY;

            % Add an intermediate roadmap point to avoid collisions.
            navigationPointer = navigationPointer - 1;
            x = roadMapObject0(1, 1);
            y = roadMapObject0(1, 2);
            navigationMatrix(navigationPointer, 1) = x;
            navigationMatrix(navigationPointer, 2) = y;
        else
            error('PATH_PLANNER: invalid object color for Object_0.')
        end

        % Final segment: the plow motion into the collection zone.
        x = collectionZonePositionObject0(objectColor, 1);
        y = collectionZonePositionObject0(objectColor, 2);
        navigationMatrix(3, 1) = x;
        navigationMatrix(3, 2) = y;
        return

    else
        % Object_1 (top slot) -------------------------------------------------
        if objectColor == 1                 % red
            % Queue PPP and collection point.
            x = collectionZonePositionObject1(objectColor, 1);
            y = collectionZonePositionObject1(objectColor, 2);
            navigationPointer = 2;
            navigationMatrix = [0 0; pppX pppY; x y];
            return

        elseif objectColor == 3             % blue
            [x, y] = DETERMINE_CLOSEST_POINT(roadMapObject1, robotX, robotY);
            if x == roadMapObject1(1, 1) && y == roadMapObject1(1, 2)
                % Approaching from the left: add a guard waypoint.
                navigationMatrix(1, 1) = -0.75;
                navigationMatrix(1, 2) = 1.9;

                navigationMatrix(2, 1) = pppX;
                navigationMatrix(2, 2) = pppY;

                x = collectionZonePositionObject1(objectColor, 1);
                y = collectionZonePositionObject1(objectColor, 2);
                navigationMatrix(3, 1) = x;
                navigationMatrix(3, 2) = y - 0.25;
                navigationPointer = 1;
            else
                x = collectionZonePositionObject1(objectColor, 1);
                y = collectionZonePositionObject1(objectColor, 2);
                navigationPointer = 2;
                navigationMatrix = [0 0; pppX pppY; x y];
            end

        elseif objectColor == 2             % green
            % Add the closest roadmap point first.
            [x, y] = DETERMINE_CLOSEST_POINT(roadMapObject1, robotX, robotY);
            navigationMatrix(1, 1) = x;
            navigationMatrix(1, 2) = y;

            % Add the pre-plow position (guarded when approaching from left).
            if x == roadMapObject1(1, 1) && y == roadMapObject1(1, 2)
                navigationMatrix(2, 1) = -0.5;
                navigationMatrix(2, 2) = 1.97;
            else
                navigationMatrix(2, 1) = pppX;
                navigationMatrix(2, 2) = pppY;
            end

            % Add the collection zone position.
            x = collectionZonePositionObject1(objectColor, 1);
            y = collectionZonePositionObject1(objectColor, 2);
            navigationMatrix(3, 1) = x;
            navigationMatrix(3, 2) = y;
            navigationPointer = 1;

        else
            error('PATH_PLANNER: invalid object color for Object_1.')
        end
    end
end
