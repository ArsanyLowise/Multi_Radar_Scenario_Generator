function [egoWaypoints, egoSpeed, truckWaypoints, truckSpeed] = createVehicleTrajectories(...
    egoStartX, truckStartX, rightLaneY, leftLaneY, roadCenters, a)
    
    % Number of waypoints from parameters
    numEgoWaypoints = a.waypointCountEgo;
    numTruckWaypoints = a.waypointAmount;
    
    % Ego waypoints
    if abs(a.roadCurvature) > 0
        % For curved road, use road centers offset by lane position
        numPoints = size(roadCenters, 1);
        
        if numPoints >= numEgoWaypoints
            % Select evenly spaced indices along the road
            indices = round(linspace(1, numPoints, numEgoWaypoints));
            egoWaypoints = zeros(numEgoWaypoints, 3);
            for j = 1:numEgoWaypoints
                egoWaypoints(j, :) = roadCenters(indices(j), :) + [0, rightLaneY, 0];
            end
        else
            % Fallback if road has fewer points
            egoWaypoints = repmat(roadCenters(1,:) + [0, rightLaneY, 0], numEgoWaypoints, 1);
        end
        
        % Create speed profile
        egoSpeed = zeros(numEgoWaypoints, 1);
        for i = 1:numEgoWaypoints
            timeInterval = a.simTime / numEgoWaypoints;
            egoSpeed(i) = a.egoSpeed + a.egoAcceleration * timeInterval * (i-1);
        end
    else
        % For straight road, create linear waypoints
        totalEgoDistance = a.roadLength - 40; % Leave margin at both ends
        if numEgoWaypoints > 1
            egoWaypointSpacing = totalEgoDistance / (numEgoWaypoints - 1);
        else
            egoWaypointSpacing = 0;
        end
        
        egoWaypoints = zeros(numEgoWaypoints, 3);
        egoSpeed = zeros(numEgoWaypoints, 1);
        for i = 1:numEgoWaypoints
            egoWaypoints(i, :) = [egoStartX + (i-1) * egoWaypointSpacing, rightLaneY, 0];
            timeInterval = a.simTime / numEgoWaypoints;
            egoSpeed(i) = a.egoSpeed + a.egoAcceleration * timeInterval * (i-1);
        end
    end

    % Target (truck) waypoints
    if abs(a.roadCurvature) > 0
        numPoints = size(roadCenters, 1);
        
        if a.targetDirection == 1
            % Same direction - target ahead of ego
            if numPoints >= numTruckWaypoints
                % Start from middle of road and go to end
                startIdx = ceil(numPoints/2) + 1;
                if startIdx + numTruckWaypoints - 1 <= numPoints
                    indices = startIdx:startIdx + numTruckWaypoints - 1;
                else
                    indices = round(linspace(startIdx, numPoints, numTruckWaypoints));
                end
            else
                indices = round(linspace(ceil(numPoints/2)+1, numPoints, numTruckWaypoints));
            end
        else
            % Opposite direction - target coming from opposite end
            if numPoints >= numTruckWaypoints
                % Go from end to beginning (reverse order)
                indices = round(linspace(numPoints, 1, numTruckWaypoints));
            else
                indices = round(linspace(numPoints, 1, numTruckWaypoints));
            end
        end
        
        % Ensure indices are within bounds
        indices = max(1, min(numPoints, indices));
        
        % Create waypoints
        truckWaypoints = zeros(numTruckWaypoints, 3);
        for j = 1:numTruckWaypoints
            truckWaypoints(j, :) = roadCenters(indices(j), :) + [0, leftLaneY, 0];
        end
        
        % Create speed profile
        truckSpeed = zeros(numTruckWaypoints, 1);
        for i = 1:numTruckWaypoints
            timeInterval = a.simTime / numTruckWaypoints;
            truckSpeed(i) = a.targetSpeed + a.targetAcceleration * timeInterval * (i-1);
        end
    else
        % For straight road
        totalTruckDistance = a.roadLength - 40;
        if numTruckWaypoints > 1
            truckWaypointSpacing = totalTruckDistance / (numTruckWaypoints - 1);
        else
            truckWaypointSpacing = 0;
        end
        
        truckWaypoints = zeros(numTruckWaypoints, 3);
        truckSpeed = zeros(numTruckWaypoints, 1);
        for i = 1:numTruckWaypoints
            if a.targetDirection == 1
                % Same direction
                truckWaypoints(i, :) = [truckStartX + (i-1) * truckWaypointSpacing, leftLaneY, 0];
            else
                % Opposite direction - go backwards
                truckWaypoints(i, :) = [truckStartX - (i-1) * truckWaypointSpacing, leftLaneY, 0];
            end
            timeInterval = a.simTime / numTruckWaypoints;
            truckSpeed(i) = a.targetSpeed + a.targetAcceleration * timeInterval * (i-1);
        end
    end
end