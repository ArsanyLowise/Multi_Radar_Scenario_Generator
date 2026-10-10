function [filteredData, stats] = processRadarData(radarData, egoPose, dbscanParams, step)
% Process radar data with DBSCAN and static removal
    filteredData = radarData;
    stats = struct('total', 0, 'static', 0, 'dynamic', 0);
    
    if ~isstruct(radarData) || ~isfield(radarData, 'Range')
        filteredData.Range = [];
        return;
    end
    
    validIdx = ~isnan(radarData.Range) & ...
               ~isnan(radarData.Azimuth) & ...
               ~isnan(radarData.RangeRate);
    
    if ~any(validIdx)
        filteredData.Range = [];
        return;
    end
    
    ranges = radarData.Range(validIdx);
    azimuths = radarData.Azimuth(validIdx);
    rangeRates = radarData.RangeRate(validIdx);
    
    % Compensate for ego motion
    egoSpeed = norm(egoPose.Velocity);
    compensatedRates = compensateEgoMotion(rangeRates, azimuths, egoSpeed);
    
    % Remove static detections
    staticIdx = abs(compensatedRates) < dbscanParams.staticThreshold;
    dynamicIdx = ~staticIdx;
    
    stats.total = length(ranges);
    stats.static = sum(staticIdx);
    stats.dynamic = sum(dynamicIdx);
    
    % Apply DBSCAN to dynamic detections only
    if any(dynamicIdx)
        X = [compensatedRates(dynamicIdx), ranges(dynamicIdx)];
        clusterIDs = enhancedDBSCAN(X, dbscanParams.epsilon, ...
            dbscanParams.minPts, dbscanParams.velocityThreshold);
        
        keepIdx = clusterIDs > 0;
        dynamicIndices = find(dynamicIdx);
        keepDynamicIndices = dynamicIndices(keepIdx);
        
        % Create filtered structure
        origIndices = find(validIdx);
        keepOrigIndices = origIndices(keepDynamicIndices);
        
        fieldNames = fieldnames(radarData);
        for f = 1:length(fieldNames)
            field = fieldNames{f};
            if length(radarData.(field)) >= max(keepOrigIndices)
                filteredData.(field) = radarData.(field)(keepOrigIndices);
            end
        end
    else
        filteredData.Range = [];
    end
end