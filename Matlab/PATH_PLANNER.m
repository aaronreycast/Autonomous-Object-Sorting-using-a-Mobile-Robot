function [navigationMatrix, navigationPointer] = PATH_PLANNER(roadMapObject0, roadMapObject1, currentObject, objectColor, robotX, robotY, pppX, pppY)

    % Collection zone positions [r; g; b]
    collectionZonePositionObject0 = [1.75 2.5; 1.9 -1.9; -2.25 0];
    collectionZonePositionObject1 = [2.2 2.2; 2.2 -2.2; -2.2 0];

    % initial values
    %collectionZonePositionObject0 = [2 2.3; 1.9 -1.9; -2.25 0];
    %collectionZonePositionObject1 = [2.2 2.2; 2.2 -2.2; -2.2 0];

    % default values
    navigationPointer = 3;
    navigationMatrix = [0 0;0 0;0 0];


    %currentObject = 0;
    if currentObject == 0

        [x, y] = DETERMINE_CLOSEST_POINT(roadMapObject0, robotX, robotY);
        if [x, y] == [pppX, pppY]
        % Add collection point to navigation. Ready to plow!
            x = collectionZonePositionObject0(objectColor,1);
            y = collectionZonePositionObject0(objectColor,2);

            % add x, y to navigation
            navigationMatrix = [0 0;0 0;x y];
            return         
        end

        % Plan a path to get to ppp  

        if objectColor == 1
            navigationPointer = navigationPointer - 1;
            navigationMatrix(navigationPointer,1) = pppX;
            navigationMatrix(navigationPointer,2) = pppY;        
        elseif objectColor == 3
            % add ppp to navigation
            navigationPointer = navigationPointer - 1;
            navigationMatrix(navigationPointer,1) = pppX;
            navigationMatrix(navigationPointer,2) = pppY;                 

            % add intermediate point to navigation
            navigationPointer = navigationPointer - 1;
            x = roadMapObject0(1,1);
            y = roadMapObject0(1,2);                
            navigationMatrix(navigationPointer,1) = x;
            navigationMatrix(navigationPointer,2) = y; 
        else
            error("error in plan planning. Object color invalid")
        end

        x = collectionZonePositionObject0(objectColor,1);
        y = collectionZonePositionObject0(objectColor,2);
        navigationMatrix(3,1) = x;
        navigationMatrix(3,2) = y;
        return


    else
        % Object 1 - WIP: similar pattern to Object0
        
        if objectColor == 1  % Red        
            % Add ppX, ppY and collection point to navigation.
            x = collectionZonePositionObject1(objectColor,1);
            y = collectionZonePositionObject1(objectColor,2);
            navigationPointer = 2;
            navigationMatrix = [0 0;pppX pppY;x y];
            return
        elseif objectColor == 3
            [x, y] = DETERMINE_CLOSEST_POINT(roadMapObject1, robotX, robotY);
            if x == roadMapObject1(1,1) && y == roadMapObject1(1,2)
                % if approaching from the left
                navigationMatrix(1,1) = -0.75;
                navigationMatrix(1,2) = 1.9;

                navigationMatrix(2,1) = pppX;
                navigationMatrix(2,2) = pppY;

                x = collectionZonePositionObject1(objectColor,1);
                y = collectionZonePositionObject1(objectColor,2);
                navigationMatrix(3,1) = x;
                navigationMatrix(3,2) = y-0.25;
                navigationPointer = 1;
            else
                % Add ppX, ppY and collection point to navigation.
                x = collectionZonePositionObject1(objectColor,1);
                y = collectionZonePositionObject1(objectColor,2);
                navigationPointer = 2;
                navigationMatrix = [0 0;pppX pppY;x y];
            end            

        elseif objectColor == 2   % green
            % Add closest point in roadmap to navigation
            [x, y] = DETERMINE_CLOSEST_POINT(roadMapObject1, robotX, robotY);

            navigationMatrix(1,1) = x;
            navigationMatrix(1,2) = y;

            % Add pre_plow position to navigation
            if x == roadMapObject1(1,1) && y == roadMapObject1(1,2)
                % if approaching from the left, use another ppt to avoid
                % collision
                navigationMatrix(2,1) = -0.5;
                navigationMatrix(2,2) = 1.97;
            else
                navigationMatrix(2,1) = pppX;
                navigationMatrix(2,2) = pppY;
            end            


            % Add collection zone position to navigation
            x = collectionZonePositionObject1(objectColor,1);
            y = collectionZonePositionObject1(objectColor,2);
            navigationMatrix(3,1) = x;
            navigationMatrix(3,2) = y; 
            navigationPointer = 1;

        else
            error('PLATH_PLANNER: objectcolor invalid')
        end        
    end
end