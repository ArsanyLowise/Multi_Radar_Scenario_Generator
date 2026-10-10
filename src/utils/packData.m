function [sensorData, sensorConfigs] = packSensorData(measurements, time, egoPose, radarSpecifications)
% Pack measurement data for tracker
    numSensors = numel(measurements);
    sensorData = struct([]);
    sensorConfigs = cell(numSensors, 1);

    for i = 1:numSensors
        pts = measurements{i};
        
        if iscell(radarSpecifications)
            radarSpec = radarSpecifications{i};
        else
            radarSpec = radarSpecifications(i);
        end
        
        sensorConfigs{i} = trackingSensorConfiguration(...
            'SensorIndex', radarSpec.SensorIndex, 'IsValidTime', true);
        
        sensorConfigs{i}.SensorLimits(1,:) = radarSpec.FieldOfView(1) * [-1/2, 1/2];
        sensorConfigs{i}.SensorLimits(2,:) = radarSpec.FieldOfView(2) * [-1/2, 1/2];
        sensorConfigs{i}.SensorLimits(3,:) = radarSpec.RangeLimits;
        sensorConfigs{i}.SensorLimits(4,:) = radarSpec.RangeRateLimits;
        
        % Sensor mounting
        sensorConfigs{i}.SensorTransformParameters(1).OriginPosition = radarSpec.MountingLocation(:);
        sensorConfigs{i}.SensorTransformParameters(1).OriginVelocity = cross(...
            radarSpec.MountingLocation(:), deg2rad(egoPose.AngularVelocity(:)));
        sensorConfigs{i}.SensorTransformParameters(1).Orientation = rotmat(...
            quaternion(radarSpec.MountingAngles(:)', 'eulerd', 'ZYX', 'frame'), 'frame');
        sensorConfigs{i}.SensorTransformParameters(1).IsParentToChild = true;
        
        % Ego pose
        sensorConfigs{i}.SensorTransformParameters(2).OriginPosition = egoPose.Position(:);
        sensorConfigs{i}.SensorTransformParameters(2).OriginVelocity = egoPose.Velocity(:);
        sensorConfigs{i}.SensorTransformParameters(2).Orientation = rotmat(...
            quaternion([egoPose.Yaw, egoPose.Pitch, egoPose.Roll], ...
            'eulerd', 'ZYX', 'frame'), 'frame');
        sensorConfigs{i}.SensorTransformParameters(2).IsParentToChild = true;
        
        sensorConfigs{i}.DetectionProbability = 0.95;
        
        % Sensor data
        sensorData(i,1).SensorIndex = sensorConfigs{i}.SensorIndex;
        sensorData(i,1).Time = time;
        
        if isstruct(pts) && isfield(pts, 'Azimuth') && ~isempty(pts.Azimuth)
            validIdx = ~isnan(pts.Azimuth) & ~isnan(pts.Elevation) & ~isnan(pts.Range);
            if any(validIdx)
                sensorData(i,1).Measurement = [pts.Azimuth(validIdx); ...
                                             pts.Elevation(validIdx); ...
                                             pts.Range(validIdx)];
            else
                sensorData(i,1).Measurement = zeros(3, 0);
            end
        else
            sensorData(i,1).Measurement = zeros(3, 0);
        end
        
        sensorData(i,1).MeasurementParameters = sensorConfigs{i}.SensorTransformParameters;
    end
end