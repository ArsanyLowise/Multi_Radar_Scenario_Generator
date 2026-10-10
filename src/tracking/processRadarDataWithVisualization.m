function [filteredFrontData, dbscanStats] = processRadarDataWithVisualization(...
    frontRadarData, egoPose, dbscanParams, step, numSteps, enableDBSCAN, ...
    dbscanStats, time, dopplerFig, rawScatter, compScatter, ax1, ax2, ...
    dbscanFig, originalScatter, compensatedScatter, staticRemovedScatter)

    filteredFrontData = frontRadarData; % Default to original
    
    if ~enableDBSCAN || ~isstruct(frontRadarData) || ~isfield(frontRadarData, 'Range')
        return;
    end
    
    validIdx = ~isnan(frontRadarData.Range) & ...
               ~isnan(frontRadarData.Azimuth) & ...
               ~isnan(frontRadarData.RangeRate);
    
    if ~any(validIdx)
        return;
    end
    
    ranges = frontRadarData.Range(validIdx);
    azimuths = frontRadarData.Azimuth(validIdx);
    rangeRates = frontRadarData.RangeRate(validIdx);
    
    % Calculate ego speed and compensate
    egoSpeed = norm(egoPose.Velocity);
    compensatedRates = compensateEgoMotion(rangeRates, azimuths, egoSpeed);
    
    % Remove static detections
    staticIdx = abs(compensatedRates) < dbscanParams.staticThreshold;
    dynamicIdx = ~staticIdx;
    
    % Update Doppler-angle visualization
    if ishandle(dopplerFig)
        set(rawScatter, 'XData', azimuths, 'YData', rangeRates, ...
            'CData', ranges, 'SizeData', 30);
        
        if any(dynamicIdx)
            set(compScatter, 'XData', azimuths(dynamicIdx), ...
                'YData', compensatedRates(dynamicIdx), ...
                'CData', ranges(dynamicIdx), 'SizeData', 30);
        else
            set(compScatter, 'XData', [], 'YData', [], 'CData', []);
        end
        
        sgtitle(dopplerFig, sprintf('Doppler-Angle Analysis - Step %d/%d', ...
            step, numSteps), 'Color', 'yellow', 'FontSize', 12);
        
        if ~isempty(ranges)
            caxis(ax1, [0 min(100, max(ranges))]);
            if any(dynamicIdx)
                caxis(ax2, [0 min(100, max(ranges(dynamicIdx)))]);
            end
        end
    end
    
    % Update DBSCAN visualization
    if exist('dbscanFig', 'var') && ishandle(dbscanFig)
        set(originalScatter, 'XData', rangeRates, 'YData', ranges);
        set(compensatedScatter, 'XData', compensatedRates, 'YData', ranges);
        
        if any(dynamicIdx)
            set(staticRemovedScatter, 'XData', compensatedRates(dynamicIdx), ...
                'YData', ranges(dynamicIdx));
        else
            set(staticRemovedScatter, 'XData', [], 'YData', []);
        end
        
        dbscanStats.time(end+1) = time;
        dbscanStats.totalPoints(end+1) = length(ranges);
        dbscanStats.afterCompensation(end+1) = length(ranges);
        dbscanStats.afterStaticRemoval(end+1) = sum(dynamicIdx);
    end
    
    % Create filtered measurement structure
    if any(dynamicIdx)
        origIndices = find(validIdx);
        keepOrigIndices = origIndices(dynamicIdx);
        
        fieldNames = fieldnames(frontRadarData);
        for f = 1:length(fieldNames)
            field = fieldNames{f};
            if length(frontRadarData.(field)) >= max(keepOrigIndices)
                filteredFrontData.(field) = frontRadarData.(field)(keepOrigIndices);
            end
        end
    else
        filteredFrontData.Range = [];
    end
end