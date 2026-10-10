clear; clc; close all; rng(2);
addpath("Functions/");

% Get optimization parameters
params = getCurrentOptimization();

%% Setup Parameters
% DBSCAN Parameters
dbscanParams = struct();
dbscanParams.epsilon = 2.0;           
dbscanParams.minPts = 2;               
dbscanParams.velocityThreshold = 1.0;   
dbscanParams.useCompensatedVelocity = true; 
dbscanParams.enableDBSCAN = true;       
dbscanParams.staticThreshold = 0.5;     

% Video Recording Parameters
recordVideos = true;                    
videoFolder = 'Videos';                  
videoFrameRate = 10;                     

% Filtering Parameters
filterDynamic = true;                    
speedThreshold = 0.5;                    

%% Setup data recording
dataFile = 'MultiRadarGridBasedTrackingData.mat';

if ~exist(dataFile, 'file')
    fprintf('Data file not found. Recording new data...\n');
    [scenario, egoVehicle, radars] = helperCreateGridBasedRadarDrivingScenario();
    helperRecordGridBasedRadarDrivingScenario(scenario, egoVehicle, radars, dataFile);
    fprintf('Data recorded and saved to %s\n', dataFile);
end

%% Load the recorded data
fprintf('Loading recorded data...\n');
load(dataFile, 'radarPointCloudLog', 'egoPoseLog', 'radarSpecification');
fprintf('Loaded %d time steps\n', numel(radarPointCloudLog));

% Recreate and display scenario
[scenario, egoVehicle, ~] = helperCreateGridBasedRadarDrivingScenario();
restart(scenario);
drivingScenarioDesigner(scenario);

%% Setup sensor configurations and tracker
sensorConfigs = createSensorConfigurations(radarSpecification, egoPoseLog);
tracker = setupTracker(sensorConfigs);

%% Setup displays
if exist('helperRadarGridTrackingDisplay', 'class') == 8
    % Enable recording for the main display
    display = helperRadarGridTrackingDisplay('Record', true);
    useDisplay = true;
    fprintf('Using helperRadarGridTrackingDisplay with recording enabled\n');
else
    useDisplay = false;
end

% Create front radar view
[frontRadarAx, targetScatter, targetInfo] = createFrontRadarView([1200, 100, 500, 600]);
drawnow; % Force initial rendering

% Create Doppler-angle visualization
[dopplerFig, rawScatter, compScatter, ax1, ax2] = createDopplerAngleView();
drawnow; % Force initial rendering
pause(0.5);

% Create DBSCAN visualization if enabled
if dbscanParams.enableDBSCAN
    [dbscanFig, originalScatter, compensatedScatter, staticRemovedScatter, axDB1, axDB2, axDB3] = ...
        createDBSCANView();
    drawnow; % Force initial rendering
    pause(0.5);
end

% Setup video recording
videoWriters = setupVideoRecording(videoFolder, videoFrameRate, recordVideos, dbscanParams.enableDBSCAN);

%% Storage for statistics
dbscanStats = struct('totalPoints', [], 'afterCompensation', [], ...
    'afterStaticRemoval', [], 'filteredPoints', [], ...
    'numClusters', [], 'time', []);

%% Main loop
numSteps = numel(radarPointCloudLog);
fprintf('Running tracker for %d steps...\n', numSteps);

trackHistory = cell(numSteps, 1);
scenarioRunning = true;
step = 1;

while step <= numSteps && scenarioRunning
    % Advance scenario
    try
        while scenario.SimulationTime < step*0.1
            if ~advance(scenario)
                scenarioRunning = false;
                fprintf('Scenario ended at step %d\n', step);
                break;
            end
        end
    catch
        scenarioRunning = false;
        break;
    end
    
    if ~scenarioRunning; break; end
    
    % Get current data
    time = step * 0.1;
    egoPose = egoPoseLog{min(step, numel(egoPoseLog))};
    measurements = radarPointCloudLog{step};
    
    % Process front radar data with DBSCAN and static removal
    frontRadarData = measurements{1};
    [filteredFrontData, dbscanStats] = processRadarDataWithVisualization(...
        frontRadarData, egoPose, dbscanParams, step, numSteps, ...
        dbscanParams.enableDBSCAN, dbscanStats, time, ...
        dopplerFig, rawScatter, compScatter, ax1, ax2, ...
        dbscanFig, originalScatter, compensatedScatter, staticRemovedScatter);
    
    % Run tracker
    [sensorData, currentConfigs] = packData(measurements, time, egoPose, radarSpecification);
    tracks = tracker(sensorData, currentConfigs, time);
    trackHistory{step} = tracks;
    
    % Update main display
    if useDisplay
        try
            display.step(scenario, egoVehicle, radarSpecification, measurements, tracker, tracks);
        catch
            % Ignore display errors
        end
    end
    
    % Update front radar view
    [~, ~] = updateFrontRadarView(frontRadarAx, targetScatter, ...
        targetInfo, filteredFrontData, step, [], filterDynamic, speedThreshold);
    
    % Capture video frames for front radar, Doppler, and DBSCAN
    if recordVideos
        captureVideoFrames(videoWriters, frontRadarAx, dopplerFig, dbscanFig, dbscanParams.enableDBSCAN, step);
    end
   
    % Progress indicator
    if mod(step, 20) == 0
        fprintf('Step %d/%d, found %d tracks\n', step, numSteps, numel(tracks));
        if dbscanParams.enableDBSCAN && ~isempty(dbscanStats.time)
            fprintf('  DBSCAN: Total:%d | Static:%d | Dynamic:%d\n', ...
                dbscanStats.totalPoints(end), ...
                dbscanStats.totalPoints(end) - dbscanStats.afterStaticRemoval(end), ...
                dbscanStats.afterStaticRemoval(end));
        end
    end
    
    drawnow limitrate;
    step = step + 1;
end

fprintf('Tracking complete! Processed %d steps.\n', step-1);

% Close the front radar, Doppler, and DBSCAN video writers
if recordVideos
    closeVideoWriters(videoWriters, dbscanParams.enableDBSCAN, numSteps);
    fprintf('Front radar, Doppler, and DBSCAN videos saved to %s folder\n', videoFolder);
end

%% Save the main display recording (this creates main_display_view.avi)
if useDisplay && recordVideos
    try
        % Create the main display video
        display.writeMovie(fullfile(videoFolder, 'main_display_view'), videoFrameRate);
        fprintf('Main display video saved to %s\\main_display_view.avi\n', videoFolder);
    catch ME
        fprintf('Could not save main display video: %s\n', ME.message);
    end
end

%% ========== COMPACT RESULTS GENERATION ==========
fprintf('\n=== Generating Results for Front Radar ===\n');

% Create Results folder
resultsFolder = 'Results';
if ~exist(resultsFolder, 'dir'); mkdir(resultsFolder); end

% Initialize storage as empty structs
radarData = struct();
Matrix = struct();

% Process each time step
for step = 1:numSteps
    time = step * 0.1;
    egoPose = egoPoseLog{step};
    frontData = radarPointCloudLog{step}{1};
    
    % Store time
    radarData.time{step,1} = time;
    Matrix.time{step,1} = time;
    
    % Initialize default empty values
    radarData.x{step,1} = [];
    radarData.y{step,1} = [];
    radarData.v{step,1} = [];
    radarData.v_compensated{step,1} = [];
    radarData.rcs{step,1} = [];
    radarData.x_nonstatic{step,1} = [];
    radarData.y_nonstatic{step,1} = [];
    radarData.rcs_nonstatic{step,1} = [];
    
    % Process radar data if available
    if isstruct(frontData) && isfield(frontData, 'Range') && any(~isnan(frontData.Range))
        validIdx = ~isnan(frontData.Range) & ~isnan(frontData.Azimuth) & ~isnan(frontData.RangeRate);
        
        if any(validIdx)
            ranges = frontData.Range(validIdx);
            azimuths = frontData.Azimuth(validIdx);
            rangeRates = frontData.RangeRate(validIdx);
            
            x = ranges .* sind(azimuths);
            y = ranges .* cosd(azimuths);
            v_comp = compensateEgoMotion(rangeRates, azimuths, norm(egoPose.Velocity));
            rcs = ones(size(ranges)); % Or use frontData.Amplitude if available
            
            nonstaticIdx = abs(v_comp) >= dbscanParams.staticThreshold;
            
            % Store data
            radarData.x{step} = x(:);
            radarData.y{step} = y(:);
            radarData.v{step} = rangeRates(:);
            radarData.v_compensated{step} = v_comp(:);
            radarData.rcs{step} = rcs(:);
            radarData.x_nonstatic{step} = x(nonstaticIdx);
            radarData.y_nonstatic{step} = y(nonstaticIdx);
            radarData.rcs_nonstatic{step} = rcs(nonstaticIdx);
        end
    end
    
    % Scenario data
    Matrix.scenarioData{step,1} = struct(...
        'roadCurvature', params.roadCurvature, ...
        'roadLength', params.roadLength, ...
        'laneWidth', params.laneWidth, ...
        'egoPosition', egoPose.Position, ...
        'egoVelocity', egoPose.Velocity, ...
        'egoYaw', egoPose.Yaw, ...
        'targetSpeed', params.targetSpeed, ...
        'targetDirection', params.targetDirection);
    
    % Copy radar data to Matrix
    Matrix.radarData.time{step,1} = time;
    Matrix.radarData.x{step,1} = radarData.x{step};
    Matrix.radarData.y{step,1} = radarData.y{step};
    Matrix.radarData.v{step,1} = radarData.v{step};
    Matrix.radarData.v_compensated{step,1} = radarData.v_compensated{step};
    Matrix.radarData.rcs{step,1} = radarData.rcs{step};
    Matrix.radarData.x_nonstatic{step,1} = radarData.x_nonstatic{step};
    Matrix.radarData.y_nonstatic{step,1} = radarData.y_nonstatic{step};
    Matrix.radarData.rcs_nonstatic{step,1} = radarData.rcs_nonstatic{step};
end

% Save files
save(fullfile(resultsFolder, 'radarData.mat'), 'radarData', '-v7.3');
save(fullfile(resultsFolder, 'Matrix.mat'), 'Matrix', '-v7.3');
fprintf('Results saved to %s folder\n', resultsFolder);

% Optional: Quick summary
totalPoints = 0;
nonstaticPoints = 0;
for step = 1:numSteps
    if ~isempty(radarData.x{step})
        totalPoints = totalPoints + length(radarData.x{step});
        nonstaticPoints = nonstaticPoints + length(radarData.x_nonstatic{step});
    end
end
fprintf('Results summary: Total detections: %d, Non-static: %d (%.1f%%)\n', ...
    totalPoints, nonstaticPoints, 100*nonstaticPoints/max(totalPoints,1));