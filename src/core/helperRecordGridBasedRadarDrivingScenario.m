function helperRecordGridBasedRadarDrivingScenario(scenario, egoVehicle, radars, fName)
% This is a helper function to record the point cloud and other relevant
% information from radars for grid-based tracking. 

rng(1);

% Restart scenario and reset sensors
restart(scenario);
for i = 1:numel(radars)
    reset(radars{i});
end

% Get radar specifications
radarSpecification = cell(numel(radars),1);
for i = 1:numel(radars)
    spec.SensorIndex = radars{i}.SensorIndex;
    spec.MountingLocation = radars{i}.MountingLocation;
    spec.MountingAngles = radars{i}.MountingAngles;
    spec.FieldOfView = radars{i}.FieldOfView;
    spec.RangeLimits = radars{i}.RangeLimits;
    spec.RangeRateLimits = radars{i}.RangeRateLimits;
    spec.DetectionProbability = 0.95;
    radarSpecification{i} = spec;
end

% Initialize point cloud and ego pose logs
radarPointCloudLog = cell(0,1);
egoPoseLog = cell(0,1);

% Sample point cloud
maxNumDets = 250;
ptCloud.Time = 0;
ptCloud.Azimuth = nan(1,maxNumDets);
ptCloud.Elevation = nan(1,maxNumDets);
ptCloud.Range = nan(1,maxNumDets);
ptCloud.RangeRate = nan(1,maxNumDets);

% Sample ego pose
egoPose.Position = zeros(1,3);
egoPose.Velocity = zeros(1,3);
egoPose.AngularVelocity = zeros(1,3);
egoPose.Yaw = 0;
egoPose.Roll = 0;
egoPose.Pitch = 0;

while advance(scenario)
    time = scenario.SimulationTime;

    tgts = targetPoses(egoVehicle);
    
    % Radar point clouds
    ptClouds = cell(numel(radars),1);
    for k = 1:numel(radars)
        detections = step(radars{k}, tgts, time);
        assert(numel(detections) <= maxNumDets,...
            'recording failed because maximum number of detections is smaller than reported detections. Modify the helper function to increase maxNumDets variable');
        
        ptCloud.Time = time;
        ptCloud.Azimuth = nan(1,maxNumDets);
        ptCloud.Elevation = nan(1,maxNumDets);
        ptCloud.Range = nan(1,maxNumDets);
        ptCloud.RangeRate = nan(1,maxNumDets);
        
        % Handle objectDetection objects
        if ~isempty(detections)
            for d = 1:numel(detections)
                if ~isempty(detections{d}) && isa(detections{d}, 'objectDetection')
                    % Extract measurements from objectDetection
                    meas = detections{d}.Measurement; % [az; el; range; rangeRate]
                    ptCloud.Azimuth(d) = meas(1);
                    ptCloud.Elevation(d) = meas(2);
                    ptCloud.Range(d) = meas(3);
                    ptCloud.RangeRate(d) = meas(4);
                end
            end
        end
        
        ptClouds{k} = ptCloud;
    end

    % Ego pose
    egoPose.Position = egoVehicle.Position;
    egoPose.Velocity = egoVehicle.Velocity;
    egoPose.AngularVelocity = egoVehicle.AngularVelocity;
    egoPose.Yaw = egoVehicle.Yaw;
    egoPose.Roll = egoVehicle.Roll;
    egoPose.Pitch = egoVehicle.Pitch;

    % Log information
    radarPointCloudLog{end+1,1} = ptClouds; %#ok<*AGROW>
    egoPoseLog{end+1,1} = egoPose;
end

save(fName, 'radarPointCloudLog', 'egoPoseLog', 'radarSpecification');
end