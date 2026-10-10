%% Bird's-Eye Ground Truth Visualization from MathWorks Example
% This creates the exact bird's-eye view visualization

clear; close all; clc;

%% Load the actual driving scenario
% This creates the exact scenario with all vehicles and their trajectories
[scenario, egoVehicle, sensors] = helperCreateGridBasedRadarDrivingScenario();

%% Load your recorded data for radar points
load('MultiRadarGridBasedTrackingData.mat', 'radarPointCloudLog', 'egoPoseLog');

%% Create the figure
fig = figure('Name', 'Bird''s-Eye Ground Truth Visualization', ...
    'Position', [100, 100, 1400, 900], ...
    'Color', [0.1 0.1 0.1]);

%% Create the layout exactly like the example
% Ground truth view (top left, larger)
axGroundTruth = subplot(3, 4, [1 2 5 6]);
set(axGroundTruth, 'Color', [0.15 0.15 0.15], ...
    'XColor', [0.9 0.9 0.9], ...
    'YColor', [0.9 0.9 0.9], ...
    'GridColor', [0.4 0.4 0.4], ...
    'GridAlpha', 0.3, ...
    'Box', 'on');
hold(axGroundTruth, 'on');
grid(axGroundTruth, 'on');
axis(axGroundTruth, 'equal');
xlabel(axGroundTruth, 'X (m)', 'Color', 'white');
ylabel(axGroundTruth, 'Y (m)', 'Color', 'white');
title(axGroundTruth, 'Ground truth - Front View', 'Color', 'white', 'FontWeight', 'bold');

% Radar views (right side)
radarTitles = {'Left Front Radar', 'Front Radar', 'Right Front Radar', ...
               'Left Rear Radar', 'Rear Radar', 'Right Rear Radar'};

axRadar = gobjects(1, 6);
axRadar(1) = subplot(3, 4, 3);  % Left Front Radar
axRadar(2) = subplot(3, 4, 4);  % Front Radar
axRadar(3) = subplot(3, 4, 7);  % Right Front Radar
axRadar(4) = subplot(3, 4, 8);  % Left Rear Radar
axRadar(5) = subplot(3, 4, 11); % Rear Radar
axRadar(6) = subplot(3, 4, 12); % Right Rear Radar

% Configure radar axes
for i = 1:6
    set(axRadar(i), 'Color', [0.15 0.15 0.15], ...
        'XColor', [0.8 0.8 0.8], ...
        'YColor', [0.8 0.8 0.8], ...
        'GridColor', [0.4 0.4 0.4], ...
        'GridAlpha', 0.3, ...
        'Box', 'on');
    grid(axRadar(i), 'on');
    hold(axRadar(i), 'on');
    axis(axRadar(i), 'equal');
    xlim(axRadar(i), [-20 60]);
    ylim(axRadar(i), [-15 15]);
    title(axRadar(i), radarTitles{i}, 'Color', 'white', 'FontSize', 9);
    plot(axRadar(i), 0, 0, 'ws', 'MarkerSize', 8, 'MarkerFaceColor', 'w');
end

% Tracker view (bottom)
axTracker = subplot(3, 4, [9 10]);
set(axTracker, 'Color', [0.1 0.1 0.1], ...
    'XColor', [0.9 0.9 0.9], ...
    'YColor', [0.9 0.9 0.9], ...
    'GridColor', [0.4 0.4 0.4], ...
    'GridAlpha', 0.3, ...
    'Box', 'on');
grid(axTracker, 'on');
hold(axTracker, 'on');
axis(axTracker, 'equal');
xlim(axTracker, [-60 60]);
ylim(axTracker, [-60 60]);
xlabel(axTracker, 'X (m)', 'Color', 'white');
ylabel(axTracker, 'Y (m)', 'Color', 'white');
title(axTracker, 'Grid-based tracker', 'Color', 'white', 'FontWeight', 'bold');

%% Create the bird's-eye view using driving.scenario.Plot
% This is the key part - using the exact same method as the example
scenarioPlotter = driving.scenario.Plot('EgoActor', egoVehicle, ...
    'Parent', axGroundTruth, ...
    'Meshes', 'on', ...
    'ViewLocation', [-11.75 0], ...  % Bird's-eye view position
    'ViewHeight', 30, ...             % Height for bird's-eye view
    'ViewPitch', 90, ...               % Look straight down (90 degrees)
    'ViewYaw', 0);                     % Looking forward

% Plot roads first
scenarioPlotter.plotRoads(scenario.RoadTiles, roadBoundaries(scenario), ...
    scenario.RoadCenters, true);

% Make roads darker for better contrast
roadPatches = findall(axGroundTruth, 'Tag', 'RoadTilesPatch');
for i = 1:numel(roadPatches)
    roadPatches(i).FaceColor = [0.2 0.2 0.2];
    roadPatches(i).EdgeColor = [0.3 0.3 0.3];
end  % Fixed: Changed } to end

%% Define colors for different elements
% Colors for vehicle types (from the example)
vehicleColors = [0.850 0.325 0.098;  % Car - orange/red
                 0.494 0.184 0.556;  % Truck - purple
                 0.466 0.674 0.188;  % Pedestrian - green
                 0.301 0.745 0.933]; % Bicycle - light blue

% Colors for radar sensors (6 distinct colors)
radarColors = lines(6);  % This gives 6 distinct colors

% Colors for ego vehicle
egoColor = [0 0.447 0.741];  % Blue

%% Simulation loop
numSteps = min(300, numel(radarPointCloudLog));
fprintf('Running bird''s-eye visualization for %d steps...\n', numSteps);

for step = 1:numSteps
    % Update scenario to current time
    while advance(scenario) && scenario.SimulationTime < step*0.1
        % Advance to correct time
    end
    
    % Get current data
    measurements = radarPointCloudLog{step};
    egoPose = egoPoseLog{step};
    
    % Clear and redraw
    cla(axGroundTruth);
    cla(axTracker);
    for i = 1:6
        cla(axRadar(i));
        plot(axRadar(i), 0, 0, 'ws', 'MarkerSize', 8, 'MarkerFaceColor', 'w');
    end
    
    %% 1. GROUND TRUTH - Bird's-eye view
    hold(axGroundTruth, 'on');
    
    % Replot roads
    scenarioPlotter.plotRoads(scenario.RoadTiles, roadBoundaries(scenario), ...
        scenario.RoadCenters, true);
    
    % Make roads darker
    roadPatches = findall(axGroundTruth, 'Tag', 'RoadTilesPatch');
    for i = 1:numel(roadPatches)
        roadPatches(i).FaceColor = [0.2 0.2 0.2];
        roadPatches(i).EdgeColor = [0.3 0.3 0.3];
    end
    
    % Plot all actors with appropriate colors
    actors = scenario.Actors;
    for i = 1:numel(actors)
        actor = actors(i);
        
        % Skip if not a valid actor
        if isempty(actor)
            continue;
        end
        
        % Determine color based on ClassID
        % ClassID: 1=car, 2=truck, 3=bicycle, 4=pedestrian
        if actor.ClassID == 1
            color = vehicleColors(1,:);  % Car
        elseif actor.ClassID == 2
            color = vehicleColors(2,:);  % Truck
        elseif actor.ClassID == 3
            color = vehicleColors(4,:);  % Bicycle
        elseif actor.ClassID == 4
            color = vehicleColors(3,:);  % Pedestrian
        else
            color = [0.5 0.5 0.5];
        end
        
        % Draw vehicle
        drawVehicle(axGroundTruth, actor.Position(1), actor.Position(2), ...
            actor.Yaw, actor.Length, actor.Width, color);
    end
    
    % Plot ego vehicle (always blue)
    drawVehicle(axGroundTruth, egoVehicle.Position(1), egoVehicle.Position(2), ...
        egoVehicle.Yaw, egoVehicle.Length, egoVehicle.Width, egoColor);
    
    % Plot radar points in ground truth - using radarColors (FIXED)
    for i = 1:6
        pts = measurements{i};
        if ~isempty(pts) && size(pts,1) > 0
            if isstruct(pts) && isfield(pts, 'Range')
                [x, y] = pol2cart(deg2rad(pts.Azimuth), pts.Range);
                ptsCart = [x(:), y(:)];
            else
                ptsCart = pts(:,1:2);
            end
            if ~isempty(ptsCart)
                scatter(axGroundTruth, ptsCart(:,1), ptsCart(:,2), 5, ...
                    radarColors(i,:), 'filled', 'MarkerFaceAlpha', 0.7);  % FIXED: changed colors to radarColors
            end
        end
    end
    
    % Set limits around ego vehicle
    xlim(axGroundTruth, [egoVehicle.Position(1)-40, egoVehicle.Position(1)+60]);
    ylim(axGroundTruth, [egoVehicle.Position(2)-35, egoVehicle.Position(2)+35]);
    
    %% 2. RADAR VIEWS - using radarColors
    for i = 1:6
        hold(axRadar(i), 'on');
        pts = measurements{i};
        if ~isempty(pts) && size(pts,1) > 0
            if isstruct(pts) && isfield(pts, 'Range')
                [x, y] = pol2cart(deg2rad(pts.Azimuth), pts.Range);
                ptsCart = [x(:), y(:)];
            else
                ptsCart = pts(:,1:2);
            end
            if ~isempty(ptsCart)
                scatter(axRadar(i), ptsCart(:,1), ptsCart(:,2), 8, ...
                    radarColors(i,:), 'filled', 'MarkerFaceAlpha', 0.8);
            end
        end
    end
    
    %% 3. TRACKER VIEW
    hold(axTracker, 'on');
    gridMap = createOccupancyGrid(measurements, 64, [-60 60 -60 60]);
    imagesc(axTracker, [-60 60], [-60 60], gridMap);
    colormap(axTracker, 'jet');
    
    % Add colorbar for tracker
    if step == 1
        colorbar(axTracker, 'Color', 'white');
    end
    
    % Update title
    sgtitle(fig, sprintf('Bird''s-Eye View - Step: %d/%d | Time: %.1f s', ...
        step, numSteps, step*0.1), 'Color', 'white', 'FontSize', 14);
    
    drawnow;
    pause(0.05);
    
    if ~isvalid(fig)
        break;
    end
    
    % Progress indicator
    if mod(step, 30) == 0
        fprintf('Processed step %d/%d\n', step, numSteps);
    end
end

fprintf('Visualization complete! Processed %d steps.\n', step);

%% Helper Functions
function drawVehicle(ax, x, y, yaw, length, width, color)
    R = [cosd(yaw) -sind(yaw); sind(yaw) cosd(yaw)];
    corners = [-length/2, -width/2; length/2, -width/2; 
               length/2, width/2; -length/2, width/2];
    corners = corners * R';
    corners = corners + [x, y];
    
    patch(ax, corners(:,1), corners(:,2), color, ...
        'FaceAlpha', 0.9, 'EdgeColor', 'white', 'LineWidth', 1);
    
    % Direction indicator
    front = [x + length/2 * cosd(yaw), y + length/2 * sind(yaw)];
    plot(ax, front(1), front(2), 'wo', 'MarkerSize', 3, 'MarkerFaceColor', 'w');
end

function gridMap = createOccupancyGrid(measurements, gridSize, limits)
    gridMap = zeros(gridSize, gridSize);
    xMin = limits(1); xMax = limits(2);
    yMin = limits(3); yMax = limits(4);
    
    for i = 1:numel(measurements)
        pts = measurements{i};
        if ~isempty(pts) && size(pts,1) > 0
            if isstruct(pts) && isfield(pts, 'Range')
                [x, y] = pol2cart(deg2rad(pts.Azimuth), pts.Range);
                ptsCart = [x(:), y(:)];
            else
                ptsCart = pts(:,1:2);
            end
            
            for j = 1:size(ptsCart,1)
                if ptsCart(j,1) >= xMin && ptsCart(j,1) <= xMax && ...
                   ptsCart(j,2) >= yMin && ptsCart(j,2) <= yMax
                   
                    xi = floor((ptsCart(j,1) - xMin) / (xMax - xMin) * (gridSize-1)) + 1;
                    yi = floor((ptsCart(j,2) - yMin) / (yMax - yMin) * (gridSize-1)) + 1;
                    
                    if xi >= 1 && xi <= gridSize && yi >= 1 && yi <= gridSize
                        gridMap(yi, xi) = gridMap(yi, xi) + 1;
                    end
                end
            end
        end
    end
    
    gridMap = imgaussfilt(gridMap, 1);
    if max(gridMap(:)) > 0
        gridMap = gridMap / max(gridMap(:));
    end
end