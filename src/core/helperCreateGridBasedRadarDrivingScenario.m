function [scenario, egoVehicle, sensors] = helperCreateGridBasedRadarDrivingScenario()


% Get optimization parameters
params = getCurrentOptimization();

% Create the drivingScenario object and ego car
[scenario, egoVehicle, targetVehicles] = createDrivingScenario(params);

% Create all the sensors
sensors = createSensors(scenario, params);

end % End of main function

function sensors = createSensors(scenario, params)
% createSensors Returns all sensor objects to generate detections

sensors{1} = radarDataGenerator('SensorIndex', 1, ...
    'MountingLocation', [3.7 0 0.6], ...
    'MountingAngles',[0 0 0]);

sensors{2} = radarDataGenerator('SensorIndex',2, ...
    'MountingLocation', [2.8 0.9 0.6], ...
    'MountingAngles',[75 0 0]);

sensors{3} = radarDataGenerator('SensorIndex',3, ...
    'MountingLocation', [2.8 0.9 0.6], ...
    'MountingAngles',[135 0 0]);

sensors{4} = radarDataGenerator('SensorIndex',4, ...
    'MountingLocation', [-1 0 0.6], ...
    'MountingAngles',[180 0 0]);

sensors{5} = radarDataGenerator('SensorIndex',5, ...
    'MountingLocation', [2.8 -0.9 0.6], ...
    'MountingAngles',[-75 0 0]);

sensors{6} = radarDataGenerator('SensorIndex',6, ...
    'MountingLocation', [2.8 -0.9 0.6], ...
    'MountingAngles',[-135 0 0]);

% Assign into each sensor the physical and radar profiles for all actors
profiles = actorProfiles(scenario);

% Radar sensor common attributes
for i = 1:numel(sensors)
    sensors{i}.ScanMode = 'No scanning';
    sensors{i}.HasElevation = true;
    sensors{i}.HasRangeRate= true;
    sensors{i}.HasNoise = true;
    sensors{i}.HasFalseAlarms = true;
    sensors{i}.UpdateRate = 1/scenario.SampleTime;
    sensors{i}.DetectionCoordinates = 'Sensor Spherical';
    sensors{i}.FieldOfView = [90 40];
    sensors{i}.RangeLimits = [0 100];
    sensors{i}.ReferenceRange = 100;
    sensors{i}.RangeRateLimits = [-100 100];
    sensors{i}.AzimuthResolution = 2;
    sensors{i}.ElevationResolution = 10;
    sensors{i}.RangeResolution = 1.5;
    sensors{i}.RangeRateResolution = 1;
    sensors{i}.FalseAlarmRate = 1e-6;
    sensors{i}.TargetReportFormat = 'Detections';
    sensors{i}.Profiles = profiles;
    sensors{i}.HasOcclusion = true;
    sensors{i}.CenterFrequency = 7.7e10;
    sensors{i}.HasGhosts = false;
    sensors{i}.HasINS = false;
end

end % End of createSensors

function [scenario, egoVehicle, targetVehicles] = createDrivingScenario(params)
% Create scenario using optimization parameters

% First create the drivingScenario object
scenario = drivingScenario('SampleTime', 0.1);

%% Define crossroad geometry
% First road (West-East) - runs roughly horizontally
roadCenters1 = [51.3 5.5 0;
                4.8 5.2 0];

% Second road (North-South) - runs roughly vertically
roadCenters2 = [30.2 27 0;
                30.2 -17.9 0];

% Create roads with 2 lanes each
laneSpecification = lanespec(params.numLanes);
road1 = road(scenario, roadCenters1, 'Lanes', laneSpecification, 'Name', 'MainRoad');
road2 = road(scenario, roadCenters2, 'Lanes', laneSpecification, 'Name', 'Road1');

%% Use barrier parameters from optimization
laneWidth = params.laneWidth;
barrierDistance = params.barrierDistance; % Distance from road edge to barrier
barrierHeight = params.barrierHeight;     % Height of barriers
barrierWidth = params.barrierWidth;       % Width of barriers

% Calculate barrier offset from road center
% Road edge is at laneWidth from center
% Barrier is placed barrierDistance beyond the road edge
barrierOffset = laneWidth + barrierDistance;

% The intersection is where the roads cross
intersectionCenter = [30.2, 5.3];

% Define intersection area (where both roads overlap)
intersectionX = [intersectionCenter(1) - laneWidth, intersectionCenter(1) + laneWidth];
intersectionY = [intersectionCenter(2) - laneWidth, intersectionCenter(2) + laneWidth];

fprintf('Using barrier parameters:\n');
fprintf('  Barrier Distance from road edge: %.2f m\n', barrierDistance);
fprintf('  Barrier Height: %.2f m\n', barrierHeight);
fprintf('  Barrier Width: %.2f m\n', barrierWidth);
fprintf('  Total offset from road center: %.2f m\n', barrierOffset);
fprintf('Intersection center: (%.2f, %.2f)\n', intersectionCenter(1), intersectionCenter(2));

%% Add continuous barriers to MAIN ROAD (West-East)
% Get road centerline points
road1Center = roadCenters1;
road1Length = sum(sqrt(sum(diff(road1Center).^2, 2)));

% Create barrier segments along Main Road
numBarriersMain = 80; 

% Pre-calculate road direction angles
numPoints1 = size(road1Center, 1);
roadAngles1 = zeros(numPoints1-1, 1);
for i = 1:numPoints1-1
    roadAngles1(i) = atan2d(road1Center(i+1,2) - road1Center(i,2), ...
                           road1Center(i+1,1) - road1Center(i,1));
end

for i = 1:numBarriersMain
    % Calculate position along the road
    t = (i-0.5) / numBarriersMain;
    
    % Find point on road centerline
    if numPoints1 >= 2
        idx = max(1, min(numPoints1-1, floor(t * (numPoints1-1)) + 1));
        nextIdx = min(numPoints1, idx + 1);
        
        % Interpolate between road points
        frac = t * (numPoints1-1) - (idx-1);
        basePos = road1Center(idx, :) + frac * (road1Center(nextIdx, :) - road1Center(idx, :));
        
        % Get road direction angle
        if idx < numPoints1
            yaw = roadAngles1(idx);
        else
            yaw = roadAngles1(end);
        end
        
        % Calculate barrier positions - using barrierOffset from optimization
        % For Main Road (horizontal), barriers are placed beyond the road edges
        
        % NORTH barrier (top side of road)
        northYaw = yaw + 90;
        northPos = [basePos(1) + barrierOffset * cosd(northYaw), ...
                    basePos(2) + barrierOffset * sind(northYaw), 0];
        
        % SOUTH barrier (bottom side of road)
        southYaw = yaw - 90;
        southPos = [basePos(1) + barrierOffset * cosd(southYaw), ...
                    basePos(2) + barrierOffset * sind(southYaw), 0];
        
        % Check if this position is INSIDE the intersection
        % Skip barriers ONLY in the intersection area
        inIntersection = (basePos(1) >= intersectionX(1) && basePos(1) <= intersectionX(2) && ...
                          basePos(2) >= intersectionY(1) && basePos(2) <= intersectionY(2));
        
        % Add barriers if NOT in intersection
        if ~inIntersection
            % North barrier
            actor(scenario, ...
                'ClassID', 5, ...
                'Position', northPos, ...
                'Yaw', yaw, ...
                'Length', road1Length/numBarriersMain * 1.5, ...
                'Width', barrierWidth, ...  % Using barrier width from params
                'Height', barrierHeight, ... % Using barrier height from params
                'Mesh', driving.scenario.jerseyBarrierMesh, ...
                'PlotColor', [0.7 0.7 0.7], ...
                'Name', sprintf('MainRoad_NorthBarrier_%d', i));
            
            % South barrier
            actor(scenario, ...
                'ClassID', 5, ...
                'Position', southPos, ...
                'Yaw', yaw, ...
                'Length', road1Length/numBarriersMain * 1.5, ...
                'Width', barrierWidth, ...
                'Height', barrierHeight, ...
                'Mesh', driving.scenario.jerseyBarrierMesh, ...
                'PlotColor', [0.7 0.7 0.7], ...
                'Name', sprintf('MainRoad_SouthBarrier_%d', i));
        end
    end
end

%% Add continuous barriers to CROSS ROAD (North-South)
% Get road centerline points
road2Center = roadCenters2;
road2Length = sum(sqrt(sum(diff(road2Center).^2, 2)));

% Pre-calculate road direction angles
numPoints2 = size(road2Center, 1);
roadAngles2 = zeros(numPoints2-1, 1);
for i = 1:numPoints2-1
    roadAngles2(i) = atan2d(road2Center(i+1,2) - road2Center(i,2), ...
                           road2Center(i+1,1) - road2Center(i,1));
end

% Create barrier segments along Cross Road
numBarriersCross = 80;

for i = 1:numBarriersCross
    % Calculate position along the road
    t = (i-0.5) / numBarriersCross;
    
    % Find point on road centerline
    if numPoints2 >= 2
        idx = max(1, min(numPoints2-1, floor(t * (numPoints2-1)) + 1));
        nextIdx = min(numPoints2, idx + 1);
        
        % Interpolate between road points
        frac = t * (numPoints2-1) - (idx-1);
        basePos = road2Center(idx, :) + frac * (road2Center(nextIdx, :) - road2Center(idx, :));
        
        % Get road direction angle
        if idx < numPoints2
            yaw = roadAngles2(idx);
        else
            yaw = roadAngles2(end);
        end
        
        % Calculate barrier positions - using barrierOffset from optimization
        % For Cross Road (vertical), barriers are placed beyond the road edges
        
        % EAST barrier (right side)
        eastYaw = yaw - 90;
        eastPos = [basePos(1) + barrierOffset * cosd(eastYaw), ...
                   basePos(2) + barrierOffset * sind(eastYaw), 0];
        
        % WEST barrier (left side)
        westYaw = yaw + 90;
        westPos = [basePos(1) + barrierOffset * cosd(westYaw), ...
                   basePos(2) + barrierOffset * sind(westYaw), 0];
        
        % Check if this position is INSIDE the intersection
        inIntersection = (basePos(1) >= intersectionX(1) && basePos(1) <= intersectionX(2) && ...
                          basePos(2) >= intersectionY(1) && basePos(2) <= intersectionY(2));
        
        % Add barriers if NOT in intersection
        if ~inIntersection
            % East barrier
            actor(scenario, ...
                'ClassID', 5, ...
                'Position', eastPos, ...
                'Yaw', yaw, ...
                'Length', road2Length/numBarriersCross * 1.5, ...
                'Width', barrierWidth, ...  % Using barrier width from params
                'Height', barrierHeight, ... % Using barrier height from params
                'Mesh', driving.scenario.jerseyBarrierMesh, ...
                'PlotColor', [0.7 0.7 0.7], ...
                'Name', sprintf('CrossRoad_EastBarrier_%d', i));
            
            % West barrier
            actor(scenario, ...
                'ClassID', 5, ...
                'Position', westPos, ...
                'Yaw', yaw, ...
                'Length', road2Length/numBarriersCross * 1.5, ...
                'Width', barrierWidth, ...
                'Height', barrierHeight, ...
                'Mesh', driving.scenario.jerseyBarrierMesh, ...
                'PlotColor', [0.7 0.7 0.7], ...
                'Name', sprintf('CrossRoad_WestBarrier_%d', i));
        end
    end
end

%% Add ego vehicle (West to East)
egoWaypoints = [6.9 3.2 0;
                13.2 3.1 0;
                19.4 3.1 0;
                24.2 3.1 0;
                30 3.3 0;
                31.3 8.5 0;
                31.9 12.4 0;
                31.9 16.7 0;
                32.1 21.7 0;
                31.9 25.6 0];

% Create constant speed profile
egoSpeed = 40/3.6; % Convert km/h to m/s
egoSpeeds = ones(size(egoWaypoints, 1), 1) * egoSpeed;

egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', egoWaypoints(1,:), ...
    'Length', 4.7, ...
    'Width', 1.8, ...
    'Height', 1.4, ...
    'FrontOverhang', 0.9, ...
    'RearOverhang', 1.1, ...
    'Wheelbase', 2.8, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Ego');

trajectory(egoVehicle, egoWaypoints, egoSpeeds);

%% Add target vehicles
targetVehicles = cell(1, 1); % Only one target as per your params

% Target 1: Coming from North to South
targetWaypoints = [28.5 23 0;
                   28.7 17.4 0;
                   28.3 12.2 0;
                   28.2 9.4 0;
                   28.3 5.9 0;
                   28.3 1.7 0;
                   28.5 -2.3 0;
                   28.3 -7.6 0;
                   28.1 -12.3 0;
                   28.1 -16.6 0];

targetSpeed = 40/3.6; % Convert km/h to m/s
targetSpeeds = ones(size(targetWaypoints, 1), 1) * targetSpeed;

targetVehicles{1} = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', targetWaypoints(1,:), ...
    'Length', 4.5, ...
    'Width', 1.8, ...
    'Height', 1.4, ...
    'FrontOverhang', 0.9, ...
    'RearOverhang', 1.1, ...
    'Wheelbase', 2.8, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Target1');

trajectory(targetVehicles{1}, targetWaypoints, targetSpeeds);

end % End of createDrivingScenario