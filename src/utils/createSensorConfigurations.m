function sensorConfigs = createSensorConfigurations(radarSpecification, egoPoseLog)
    sensorConfigs = cell(numel(radarSpecification), 1);
    for i = 1:numel(sensorConfigs)
        if iscell(radarSpecification)
            radarSpec = radarSpecification{i};
        else
            radarSpec = radarSpecification(i);
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
            radarSpec.MountingLocation(:), deg2rad(egoPoseLog{1}.AngularVelocity(:)));
        sensorConfigs{i}.SensorTransformParameters(1).Orientation = rotmat(...
            quaternion(radarSpec.MountingAngles(:)', 'eulerd', 'ZYX', 'frame'), 'frame');
        sensorConfigs{i}.SensorTransformParameters(1).IsParentToChild = true;
        
        % Ego pose
        sensorConfigs{i}.SensorTransformParameters(2).OriginPosition = egoPoseLog{1}.Position(:);
        sensorConfigs{i}.SensorTransformParameters(2).OriginVelocity = egoPoseLog{1}.Velocity(:);
        sensorConfigs{i}.SensorTransformParameters(2).Orientation = rotmat(...
            quaternion([egoPoseLog{1}.Yaw, egoPoseLog{1}.Pitch, egoPoseLog{1}.Roll], ...
            'eulerd', 'ZYX', 'frame'), 'frame');
        sensorConfigs{i}.SensorTransformParameters(2).IsParentToChild = true;
        
        sensorConfigs{i}.DetectionProbability = 0.95;
    end
end