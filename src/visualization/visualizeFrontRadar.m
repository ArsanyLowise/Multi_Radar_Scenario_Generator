function visualizeFrontRadar()
% Separate visualization for front radar detections only
% Uses DBSCAN clustering and target filtering to show only relevant detections
% X-axis: -10 to 10, Y-axis: 0 to 60

% Load the recorded data
fprintf('Loading recorded data...\n');
load('MultiRadarGridBasedTrackingData.mat', 'radarPointCloudLog', 'egoPoseLog');
fprintf('Loaded %d time steps\n', numel(radarPointCloudLog));

% Get optimization parameters
params = getCurrentOptimization();
targetDirection = params.targetDirection; % -1 for opposite, 1 for same

% Create figure for front radar visualization
fig = figure('Name', 'Front Radar Detection Visualization', ...
    'Position', [100, 100, 1000, 800], ...
    'Color', [0.1 0.1 0.1]);

% Create axes for front radar - updated limits
ax = axes('Parent', fig, ...
    'Color', [0.15 0.15 0.15], ...
    'XColor', [0.9 0.9 0.9], ...
    'YColor', [0.9 0.9 0.9], ...
    'GridColor', [0.4 0.4 0.4], ...
    'GridAlpha', 0.3, ...
    'Box', 'on');
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
xlim(ax, [-10 10]);     % Updated: -10 to 10
ylim(ax, [0 60]);       % Updated: 0 to 60
xlabel(ax, 'Lateral Distance (m)', 'Color', 'white', 'FontSize', 12);
ylabel(ax, 'Forward Distance (m)', 'Color', 'white', 'FontSize', 12);
title(ax, 'Front Radar - Target Detections', 'Color', 'white', 'FontWeight', 'bold', 'FontSize', 14);

% Plot ego vehicle
egoMarker = plot(ax, 0, 0, 'ws', 'MarkerSize', 15, 'MarkerFaceColor', 'w', 'LineWidth', 2);
text(ax, 0, 2, 'EGO', 'Color', 'white', 'HorizontalAlignment', 'center', ...
    'FontWeight', 'bold', 'FontSize', 10);

% Add range rings (simplified)
for r = [20, 30]
    % Only draw within y-limits
    if r <= 30
        theta_arc = linspace(-pi/2, pi/2, 100);
        x_arc = r * sin(theta_arc);
        y_arc = r * cos(theta_arc);
        % Only keep points within x limits
        valid_idx = abs(x_arc) <= 10;
        plot(ax, x_arc(valid_idx), y_arc(valid_idx), ...
            'Color', [0.5 0.5 0.5], 'LineWidth', 0.5, 'LineStyle', ':');
    end
end

% Add lane markers (simplified)
laneWidth = 3.5;
% Target lane (left)
line(ax, [-laneWidth/2, -laneWidth/2], [0, 60], ...
    'Color', 'cyan', 'LineWidth', 1.5, 'LineStyle', '-');
% Ego lane (right)
line(ax, [laneWidth/2, laneWidth/2], [0, 60], ...
    'Color', [0.8 0.8 0.8], 'LineWidth', 1, 'LineStyle', '--');
% Center line
line(ax, [0, 0], [0, 60], 'Color', [0.98 0.86 0.36], 'LineWidth', 1.5, 'LineStyle', '-');

% Title for step info (no extra rectangle)
stepTitle = title(ax, '', 'Color', 'yellow', 'FontSize', 11, 'FontWeight', 'bold');

%% Main loop
numSteps = numel(radarPointCloudLog);
fprintf('Visualizing front radar detections for %d steps...\n', numSteps);

% Storage for tracking
targetDetected = false(numSteps, 1);
targetDistance = zeros(numSteps, 1);
targetSpeed = zeros(numSteps, 1);
targetLateral = zeros(numSteps, 1);

% Scatter plot for final filtered detections
filteredScatter = scatter(ax, [], [], 40, 'red', 'filled', 'MarkerEdgeColor', 'white');

% Simple text for target info (no background rectangle)
targetText = text(ax, 0, 0, '', 'Color', 'cyan', 'FontSize', 10, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'center');

% Target tracking variables
expectedTruckPos = []; % Will be initialized from first detection
targetSearchRadius = 5; % Search radius around expected position
minTargetDetections = 3; % Minimum detections to confirm target
targetMemory = []; % Store previous target positions for smoothing

% DBSCAN parameters
epsilon = 2.0; % Distance threshold for clustering (meters)
MinPts = 2;    % Minimum points for core point
velocityThreshold = 1.0; % Velocity threshold for merging clusters (m/s)

%% SCAN FOR INITIAL TARGET POSITION
fprintf('Scanning for initial target position...\n');
allTargetCandidates = [];

for scanStep = 1:min(20, numSteps) % Look at first 20 steps
    measurements = radarPointCloudLog{scanStep};
    frontRadarData = measurements{1};
    
    if isstruct(frontRadarData) && isfield(frontRadarData, 'Range')
        validIdx = ~isnan(frontRadarData.Range) & ...
                   ~isnan(frontRadarData.Azimuth) & ...
                   ~isnan(frontRadarData.RangeRate);
        
        if any(validIdx)
            ranges = frontRadarData.Range(validIdx);
            azimuths = frontRadarData.Azimuth(validIdx);
            rangeRates = frontRadarData.RangeRate(validIdx);
            
            x_all = ranges .* sind(azimuths);
            y_all = ranges .* cosd(azimuths);
            
            % Look for moving objects in left lane
            inLeftLane = x_all > -3.5 & x_all < 0;
            isMoving = abs(rangeRates) > 1.0;
            validDistance = y_all > 2 & y_all < 50;
            
            candidates = inLeftLane & isMoving & validDistance;
            
            if any(candidates)
                % Get candidate points
                x_cand = x_all(candidates);
                y_cand = y_all(candidates);
                
                % Make sure they are column vectors
                if size(x_cand, 1) < size(x_cand, 2)
                    x_cand = x_cand';
                end
                if size(y_cand, 1) < size(y_cand, 2)
                    y_cand = y_cand';
                end
                
                % Append to candidates list
                allTargetCandidates = [allTargetCandidates; [x_cand, y_cand]];
            end
        end
    end
end

% Initialize expected truck position
if ~isempty(allTargetCandidates)
    expectedTruckPos = [median(allTargetCandidates(:,1)), median(allTargetCandidates(:,2))];
    fprintf('Initial target position estimate: X=%.2f m, Y=%.2f m (from %d candidates)\n', ...
        expectedTruckPos(1), expectedTruckPos(2), size(allTargetCandidates, 1));
else
    % Fallback - look in left lane at moderate distance
    expectedTruckPos = [-1.75, 30];
    fprintf('No candidates found. Using fallback target position: X=%.2f m, Y=%.2f m\n', ...
        expectedTruckPos(1), expectedTruckPos(2));
end

%% MAIN PROCESSING LOOP
for step = 1:numSteps
    % Get front radar measurements
    measurements = radarPointCloudLog{step};
    frontRadarData = measurements{1};
    
    if isstruct(frontRadarData) && isfield(frontRadarData, 'Range')
        % Remove NaN values
        validIdx = ~isnan(frontRadarData.Range) & ...
                   ~isnan(frontRadarData.Azimuth) & ...
                   ~isnan(frontRadarData.RangeRate);
        
        if any(validIdx)
            ranges = frontRadarData.Range(validIdx);
            azimuths = frontRadarData.Azimuth(validIdx);
            rangeRates = frontRadarData.RangeRate(validIdx);
            
            % Convert to Cartesian
            x_all = ranges .* sind(azimuths);
            y_all = ranges .* cosd(azimuths);
            
            %% Step 1: Apply DBSCAN clustering
            X = [rangeRates(:), ranges(:)];
            clusterIDs = enhancedDBSCAN(X, epsilon, MinPts, velocityThreshold);
            
            %% Step 2: Apply simple DBSCAN filtering
            [filteredVel, filteredRange, clustersToKeep] = ...
                simpleDBSCANFiltering(rangeRates, ranges, clusterIDs);
            
            if ~isempty(filteredVel)
                % Get indices of points to keep (all non-noise)
                keepIdx = clusterIDs > 0;
                x_dbscan = x_all(keepIdx);
                y_dbscan = y_all(keepIdx);
                rr_dbscan = rangeRates(keepIdx);
                
                %% Step 3: Create objectDetection-like struct
                filteredDets = {};
                for i = 1:length(x_dbscan)
                    det.Measurement = [x_dbscan(i); y_dbscan(i)];
                    det.MeasurementNoise = eye(2) * 0.1;
                    filteredDets{end+1} = det;
                end
                
                %% Step 4: Apply target-focused filtering
                totalDets = length(x_all);
                finalFilteredDets = applyTargetFocusFilteringSingleTarget(...
                    filteredDets, expectedTruckPos, targetSearchRadius, ...
                    minTargetDetections, step*0.1, totalDets);
                
                % Extract final filtered detections
                if ~isempty(finalFilteredDets)
                    x_final = zeros(length(finalFilteredDets), 1);
                    y_final = zeros(length(finalFilteredDets), 1);
                    rr_final = zeros(length(finalFilteredDets), 1);
                    
                    for i = 1:length(finalFilteredDets)
                        x_final(i) = finalFilteredDets{i}.Measurement(1);
                        y_final(i) = finalFilteredDets{i}.Measurement(2);
                        
                        % Find corresponding range rate by matching position
                        [~, closestIdx] = min(abs(x_dbscan - x_final(i)) + abs(y_dbscan - y_final(i)));
                        if ~isempty(closestIdx)
                            rr_final(i) = rr_dbscan(closestIdx(1));
                        end
                    end
                    
                    % Update scatter plot
                    set(filteredScatter, 'XData', x_final, 'YData', y_final);
                    
                    % Update expected truck position
                    if length(x_final) > 0
                        newX = median(x_final);
                        newY = median(y_final);
                        
                        if isempty(targetMemory)
                            expectedTruckPos = [newX, newY];
                            targetMemory = [newX, newY];
                        else
                            % Smooth with previous position
                            expectedTruckPos = [0.7*newX + 0.3*targetMemory(1), ...
                                                0.7*newY + 0.3*targetMemory(2)];
                            targetMemory = [newX, newY];
                        end
                        
                        targetDetected(step) = true;
                        targetDistance(step) = expectedTruckPos(2);
                        targetSpeed(step) = median(abs(rr_final));
                        targetLateral(step) = expectedTruckPos(1);
                        
                        set(targetText, 'Position', [expectedTruckPos(1), expectedTruckPos(2)+4, 0], ...
                            'String', sprintf('TARGET\n%d det | %.1f m | %.1f m/s', ...
                            length(x_final), expectedTruckPos(2), median(abs(rr_final))));
                    end
                else
                    set(filteredScatter, 'XData', [], 'YData', []);
                    set(targetText, 'String', '');
                    targetDetected(step) = false;
                    targetSearchRadius = min(15, targetSearchRadius * 1.1);
                end
            else
                set(filteredScatter, 'XData', [], 'YData', []);
                set(targetText, 'String', '');
                targetDetected(step) = false;
            end
        else
            set(filteredScatter, 'XData', [], 'YData', []);
            set(targetText, 'String', '');
            targetDetected(step) = false;
        end
    else
        set(filteredScatter, 'XData', [], 'YData', []);
        set(targetText, 'String', '');
        targetDetected(step) = false;
    end
    
    % Update title (simple)
    if targetDetected(step)
        set(stepTitle, 'String', sprintf('Step: %d/%d | TARGET | Dist: %.1f m', ...
            step, numSteps, targetDistance(step)));
    else
        set(stepTitle, 'String', sprintf('Step: %d/%d | NO TARGET', ...
            step, numSteps));
    end
    
    drawnow;
    pause(0.05);
    
    if mod(step, 20) == 0 && targetDetected(step)
        fprintf('Step %d/%d: Target at %.1f m, %.1f m/s\n', ...
            step, numSteps, targetDistance(step), targetSpeed(step));
    end
    
    if ~isvalid(fig)
        break;
    end
end

% Statistics
fprintf('\n=== TARGET DETECTION SUMMARY ===\n');
fprintf('Target detected in: %.1f%% of steps (%d/%d)\n', ...
    100 * sum(targetDetected)/numSteps, sum(targetDetected), numSteps);

if sum(targetDetected) > 0
    validDistances = targetDistance(targetDetected);
    validSpeeds = targetSpeed(targetDetected);
    fprintf('Distance - Min: %.1f m, Max: %.1f m, Avg: %.1f m\n', ...
        min(validDistances), max(validDistances), mean(validDistances));
    fprintf('Speed - Min: %.1f m/s, Max: %.1f m/s, Avg: %.1f m/s\n', ...
        min(validSpeeds), max(validSpeeds), mean(validSpeeds));
end



end

