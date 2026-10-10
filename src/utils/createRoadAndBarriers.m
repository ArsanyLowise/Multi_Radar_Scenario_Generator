function [roadCenters, rightLaneY, leftLaneY] = createRoadAndBarriers(scenario, a)
    % Road Definition
    if abs(a.roadCurvature) > 0
        radius = 100 / deg2rad(abs(a.roadCurvature));
        theta = linspace(0, deg2rad(abs(a.roadCurvature)), a.wayPointAmount);
        if a.roadCurvature > 0
            roadCenters = [radius * sin(theta)', radius * (1 - cos(theta))', zeros(length(theta), 1)];
        else
            roadCenters = [radius * sin(theta)', -radius * (1 - cos(theta))', zeros(length(theta), 1)];
        end
        % Scale to desired length
        currentLength = sum(sqrt(sum(diff(roadCenters).^2, 2)));
        scaleFactor = a.roadLength / currentLength;
        roadCenters = roadCenters * scaleFactor;
    else
        roadCenters = [0 0 0; a.roadLength 0 0];
    end

    laneWidth = a.laneWidth;
    rightLaneY = -laneWidth/2;
    leftLaneY = laneWidth/2;

    laneSpecification = lanespec(2);
    road1 = road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'MainRoad');

    % Add barriers using actors
    % Number of barrier segments
    numBarriers = 30; % Increased for smoother appearance
    
    % Create barriers along the road
    for i = 1:numBarriers
        % Find position along the road
        t = (i-0.5) / numBarriers; % Position parameter (0 to 1)
        
        % Interpolate to find point on road
        if size(roadCenters, 1) >= 2
            % Simple linear interpolation along the road
            totalLength = size(roadCenters, 1);
            idx = max(1, min(totalLength-1, floor(t * (totalLength-1)) + 1));
            nextIdx = min(totalLength, idx + 1);
            
            % Interpolate between road points
            frac = t * (totalLength-1) - (idx-1);
            
            if idx < size(roadCenters, 1)
                basePos = roadCenters(idx, :) + frac * (roadCenters(nextIdx, :) - roadCenters(idx, :));
            else
                basePos = roadCenters(end, :);
            end
            
            % Calculate direction for orientation
            if idx < size(roadCenters, 1)
                dir = roadCenters(nextIdx, :) - roadCenters(idx, :);
                yaw = atan2d(dir(2), dir(1));
            else
                yaw = 0;
            end
            
            % Calculate perpendicular direction for barrier offset
            % Rotate the direction vector by 90 degrees to get perpendicular
            % For a 2D vector (dx, dy), perpendicular is (-dy, dx) or (dy, -dx)
            dx = cosd(yaw);
            dy = sind(yaw);
            
            % Perpendicular vector (rotate 90 degrees clockwise for right side)
            % Right side perpendicular: (dy, -dx)
            % Left side perpendicular: (-dy, dx)
            
            % Distance from road center to barrier (road edge + barrier distance)
            % Road edge is at laneWidth/2 from center
            offsetDistance = laneWidth/2 + a.barrierDistance + a.barrierWidth/2;
            
            % Right barrier (positive perpendicular direction)
            rightOffsetX = dy * offsetDistance;
            rightOffsetY = -dx * offsetDistance;
            rightBarrierPos = basePos + [rightOffsetX, rightOffsetY, 0];
            
            % Add right barrier
            if all(isfinite(rightBarrierPos))
                actor(scenario, ...
                    'ClassID', 5, ...
                    'Position', rightBarrierPos, ...
                    'Yaw', yaw, ...
                    'Length', a.roadLength/numBarriers * 1.2, ... % Slightly longer to ensure overlap
                    'Width', a.barrierWidth, ...
                    'Height', a.barrierHeight, ...
                    'Mesh', driving.scenario.jerseyBarrierMesh, ...
                    'Name', sprintf('RightBarrier_%d', i));
            end
            
            % Left barrier (negative perpendicular direction)
            leftOffsetX = -dy * offsetDistance;
            leftOffsetY = dx * offsetDistance;
            leftBarrierPos = basePos + [leftOffsetX, leftOffsetY, 0];
            
            % Add left barrier
            if all(isfinite(leftBarrierPos))
                actor(scenario, ...
                    'ClassID', 5, ...
                    'Position', leftBarrierPos, ...
                    'Yaw', yaw, ...
                    'Length', a.roadLength/numBarriers * 1.2, ... % Slightly longer to ensure overlap
                    'Width', a.barrierWidth, ...
                    'Height', a.barrierHeight, ...
                    'Mesh', driving.scenario.jerseyBarrierMesh, ...
                    'Name', sprintf('LeftBarrier_%d', i));
            end
        end
    end
end