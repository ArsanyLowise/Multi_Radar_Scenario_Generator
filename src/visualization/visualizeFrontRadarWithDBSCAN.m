function visualizeFrontRadarWithDBSCAN(radarPointCloudLog, egoPoseLog, params, dbscanParams)
% Separate visualization script for front radar with DBSCAN clustering
% This runs independently from the main tracking

    fprintf('Starting front radar visualization with DBSCAN...\n');
    
    % Create figure for front radar visualization
    fig = figure('Name', 'Front Radar with DBSCAN Clustering', ...
        'Position', [100, 100, 1400, 900], ...
        'Color', [0.1 0.1 0.1]);

    % Create main axes for radar view
    ax = axes('Parent', fig, ...
        'Position', [0.1, 0.1, 0.55, 0.8], ...
        'Color', [0.15 0.15 0.15], ...
        'XColor', [0.9 0.9 0.9], ...
        'YColor', [0.9 0.9 0.9], ...
        'GridColor', [0.4 0.4 0.4], ...
        'GridAlpha', 0.3, ...
        'Box', 'on');
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    xlim(ax, [-10 10]);
    ylim(ax, [0 60]);
    xlabel(ax, 'Lateral Distance (m)', 'Color', 'white', 'FontSize', 12);
    ylabel(ax, 'Forward Distance (m)', 'Color', 'white', 'FontSize', 12);
    title(ax, 'Front Radar - DBSCAN Clustering', 'Color', 'white', 'FontWeight', 'bold', 'FontSize', 14);

    % Create axes for DBSCAN analysis (velocity-range plot)
    axDBSCAN = axes('Parent', fig, ...
        'Position', [0.7, 0.6, 0.25, 0.3], ...
        'Color', [0.15 0.15 0.15], ...
        'XColor', [0.9 0.9 0.9], ...
        'YColor', [0.9 0.9 0.9], ...
        'GridColor', [0.4 0.4 0.4], ...
        'Box', 'on');
    hold(axDBSCAN, 'on');
    grid(axDBSCAN, 'on');
    xlabel(axDBSCAN, 'Radial Velocity (m/s)', 'Color', 'white');
    ylabel(axDBSCAN, 'Range (m)', 'Color', 'white');
    title(axDBSCAN, 'Velocity-Range Space', 'Color', 'white');

    % Create axes for statistics
    axStats = axes('Parent', fig, ...
        'Position', [0.7, 0.1, 0.25, 0.4], ...
        'Color', [0.15 0.15 0.15], ...
        'XColor', [0.9 0.9 0.9], ...
        'YColor', [0.9 0.9 0.9], ...
        'Box', 'on');
    hold(axStats, 'on');
    grid(axStats, 'on');
    xlabel(axStats, 'Time (s)', 'Color', 'white');
    ylabel(axStats, 'Count', 'Color', 'white');
    title(axStats, 'DBSCAN Statistics', 'Color', 'white');

    % Plot ego vehicle
    plot(ax, 0, 0, 'ws', 'MarkerSize', 15, 'MarkerFaceColor', 'w', 'LineWidth', 2);
    text(ax, 0, 2, 'EGO', 'Color', 'white', 'HorizontalAlignment', 'center', ...
        'FontWeight', 'bold', 'FontSize', 10);

    % Add lane markers
    laneWidth = 3.5;
    line(ax, [-laneWidth/2, -laneWidth/2], [0, 60], ...
        'Color', 'cyan', 'LineWidth', 1.5);
    line(ax, [laneWidth/2, laneWidth/2], [0, 60], ...
        'Color', [0.8 0.8 0.8], 'LineWidth', 1, 'LineStyle', '--');
    line(ax, [0, 0], [0, 60], 'Color', [0.98 0.86 0.36], 'LineWidth', 1.5);

    % Scatter plots for different stages
    rawScatter = scatter(ax, [], [], 20, 'blue', 'filled', 'MarkerFaceAlpha', 0.3);
    clusteredScatter = scatter(ax, [], [], 40, 'red', 'filled', 'MarkerEdgeColor', 'white');
    
    % Scatter for velocity-range plot
    velRangeScatter = scatter(axDBSCAN, [], [], 30, 'blue', 'filled');
    filteredVelRangeScatter = scatter(axDBSCAN, [], [], 40, 'red', 'filled', 'MarkerEdgeColor', 'white');

    % Storage for statistics
    stats.time = [];
    stats.totalPoints = [];
    stats.filteredPoints = [];
    stats.numClusters = [];
    
    % Text for information
    infoText = text(ax, 5, 55, '', 'Color', 'yellow', 'FontSize', 10, ...
        'FontWeight', 'bold', 'VerticalAlignment', 'top');
    
    % Step title
    stepTitle = title(ax, '', 'Color', 'yellow', 'FontSize', 11, 'FontWeight', 'bold');

    % Main loop
    numSteps = numel(radarPointCloudLog);
    
    for step = 1:numSteps
        % Get front radar data
        measurements = radarPointCloudLog{step};
        frontData = measurements{1};
        egoPose = egoPoseLog{step};
        
        if isstruct(frontData) && isfield(frontData, 'Range')
            % Extract valid measurements
            validIdx = ~isnan(frontData.Range) & ...
                       ~isnan(frontData.Azimuth) & ...
                       ~isnan(frontData.RangeRate);
            
            if any(validIdx)
                ranges = frontData.Range(validIdx);
                azimuths = frontData.Azimuth(validIdx);
                rangeRates = frontData.RangeRate(validIdx);
                
                % Convert to Cartesian
                x_all = ranges .* sind(azimuths);
                y_all = ranges .* cosd(azimuths);
                
                % Calculate compensated velocity (simplified)
                egoSpeed = norm(egoPose.Velocity);
                cosAngle = cosd(azimuths);
                compensatedRates = rangeRates + egoSpeed .* cosAngle;
                
                % Apply DBSCAN
                X = [compensatedRates(:), ranges(:)];
                clusterIDs = enhancedDBSCAN(X, dbscanParams.epsilon, ...
                    dbscanParams.minPts, dbscanParams.velocityThreshold);
                
                % Filter points
                keepIdx = clusterIDs > 0;
                x_filtered = x_all(keepIdx);
                y_filtered = y_all(keepIdx);
                vel_filtered = compensatedRates(keepIdx);
                range_filtered = ranges(keepIdx);
                
                % Update scatter plots
                set(rawScatter, 'XData', x_all, 'YData', y_all);
                set(clusteredScatter, 'XData', x_filtered, 'YData', y_filtered);
                
                % Update velocity-range plot
                set(velRangeScatter, 'XData', compensatedRates, 'YData', ranges);
                set(filteredVelRangeScatter, 'XData', vel_filtered, 'YData', range_filtered);
                
                % Store statistics
                stats.time(end+1) = step * 0.1;
                stats.totalPoints(end+1) = length(ranges);
                stats.filteredPoints(end+1) = sum(keepIdx);
                stats.numClusters(end+1) = length(unique(clusterIDs(keepIdx)));
                
                % Update statistics plot (keep last 100 points for performance)
                if length(stats.time) > 100
                    plotIdx = length(stats.time)-99:length(stats.time);
                else
                    plotIdx = 1:length(stats.time);
                end
                
                cla(axStats);
                plot(axStats, stats.time(plotIdx), stats.totalPoints(plotIdx), 'b-', 'LineWidth', 1.5);
                plot(axStats, stats.time(plotIdx), stats.filteredPoints(plotIdx), 'r-', 'LineWidth', 1.5);
                plot(axStats, stats.time(plotIdx), stats.numClusters(plotIdx), 'g-', 'LineWidth', 1.5);
                legend(axStats, {'Total', 'Filtered', 'Clusters'}, 'TextColor', 'white', 'Location', 'best');
                
                % Update info text
                set(infoText, 'String', sprintf(...
                    'Total: %d | Filtered: %d | Clusters: %d | Keep: %.1f%%', ...
                    length(ranges), sum(keepIdx), length(unique(clusterIDs(keepIdx))), ...
                    100*sum(keepIdx)/length(ranges)));
                
                set(stepTitle, 'String', sprintf('Step: %d/%d | Time: %.1fs', ...
                    step, numSteps, step*0.1));
            else
                set(rawScatter, 'XData', [], 'YData', []);
                set(clusteredScatter, 'XData', [], 'YData', []);
                set(velRangeScatter, 'XData', [], 'YData', []);
                set(filteredVelRangeScatter, 'XData', [], 'YData', []);
            end
        end
        
        drawnow;
        pause(0.05);
        
        if ~isvalid(fig)
            break;
        end
        
        if mod(step, 50) == 0
            fprintf('Visualization step %d/%d\n', step, numSteps);
        end
    end
    
    % Final statistics
    if ~isempty(stats.time)
        fprintf('\n=== DBSCAN Statistics ===\n');
        fprintf('Average points per frame: %.1f\n', mean(stats.totalPoints));
        fprintf('Average filtered points: %.1f\n', mean(stats.filteredPoints));
        fprintf('Average clusters: %.1f\n', mean(stats.numClusters));
        fprintf('Average retention rate: %.1f%%\n', 100*mean(stats.filteredPoints./stats.totalPoints));
    end
end