function [targetPos, targetDetected] = updateFrontRadarView(frontAx, targetScatter, targetInfo, frontData, step, params, filterDynamic, speedThreshold)
% updateFrontRadarView - Updates the front radar visualization
% 
% Inputs:
%   frontAx       - axes handle
%   targetScatter - scatter handle
%   targetInfo    - text handle
%   frontData     - radar measurements
%   step          - current step number
%   params        - not used (kept for compatibility)
%   filterDynamic - boolean: true = show only dynamic detections
%   speedThreshold - minimum speed to consider as dynamic (m/s)

    targetPos = [NaN, NaN];
    targetDetected = false;
    
    % Set defaults if not provided
    if nargin < 7
        filterDynamic = false;
    end
    if nargin < 8
        speedThreshold = 0.5;
    end
    
    try
        if ~isstruct(frontData) || ~isfield(frontData, 'Range')
            set(targetScatter, 'XData', [], 'YData', []);
            set(targetInfo, 'String', '');
            title(frontAx, sprintf('Step %d | No data', step), 'Color', 'yellow');
            return;
        end
        
        % Get valid indices (remove NaNs)
        valid = ~isnan(frontData.Range) & ~isnan(frontData.Azimuth);
        
        % Also need range rate for dynamic filtering
        if filterDynamic
            valid = valid & ~isnan(frontData.RangeRate);
        end
        
        if ~any(valid)
            set(targetScatter, 'XData', [], 'YData', []);
            set(targetInfo, 'String', '');
            title(frontAx, sprintf('Step %d | No valid', step), 'Color', 'yellow');
            return;
        end
        
        % Convert to Cartesian coordinates
        x = frontData.Range(valid) .* sind(frontData.Azimuth(valid));
        y = frontData.Range(valid) .* cosd(frontData.Azimuth(valid));
        
        % Filter by view limits
        inView = abs(x) <= 10 & y >= 0 & y <= 30;
        x = x(inView);
        y = y(inView);
        
        % If dynamic filtering is enabled, also filter by range rate
        if filterDynamic
            rangeRates = frontData.RangeRate(valid);
            rangeRates = rangeRates(inView);
            
            % Keep only detections with significant range rate (moving objects)
            isDynamic = abs(rangeRates) > speedThreshold;
            x = x(isDynamic);
            y = y(isDynamic);
            
            % Update plot with dynamic count
            set(targetScatter, 'XData', x, 'YData', y);
            set(targetInfo, 'String', sprintf('%d dynamic (>=%.1f m/s)', length(x), speedThreshold));
            title(frontAx, sprintf('Step %d | %d dynamic', step, length(x)), 'Color', 'white');
        else
            % Show all detections
            set(targetScatter, 'XData', x, 'YData', y);
            set(targetInfo, 'String', sprintf('%d total', length(x)));
            title(frontAx, sprintf('Step %d | %d total', step, length(x)), 'Color', 'white');
        end
        
        if ~isempty(x)
            targetDetected = true;
            targetPos = [median(x), median(y)];
        end
        
    catch ME
        fprintf('Error at step %d: %s\n', step, ME.message);
    end
end