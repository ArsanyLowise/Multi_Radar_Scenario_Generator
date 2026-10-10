function [dbscanFig, originalScatter, compensatedScatter, staticRemovedScatter, axDB1, axDB2, axDB3] = createDBSCANView()
    dbscanFig = figure('Name', 'DBSCAN Clustering Results - Dynamic Targets Only', ...
        'Position', [100, 100, 1400, 400], 'Color', [0.1 0.1 0.1]);
    
    % Left subplot - Original detections
    axDB1 = subplot(1,3,1);
    setupDBSCANAxes(axDB1, 'Original Detections', 'Radial Velocity (m/s)', 'Range (m)');
    
    % Middle subplot - After ego compensation
    axDB2 = subplot(1,3,2);
    setupDBSCANAxes(axDB2, 'After Ego Compensation', 'Compensated Velocity (m/s)', 'Range (m)');
    
    % Right subplot - After static removal
    axDB3 = subplot(1,3,3);
    setupDBSCANAxes(axDB3, 'After Static Removal (Dynamic Only)', 'Compensated Velocity (m/s)', 'Range (m)');
    
    % Scatter plots
    originalScatter = scatter(axDB1, [], [], 15, 'blue', 'filled', 'MarkerFaceAlpha', 0.5);
    compensatedScatter = scatter(axDB2, [], [], 15, 'cyan', 'filled', 'MarkerFaceAlpha', 0.5);
    staticRemovedScatter = scatter(axDB3, [], [], 20, 'green', 'filled', 'MarkerEdgeColor', 'white', 'MarkerFaceAlpha', 0.7);
end

function setupDBSCANAxes(ax, titleStr, xlabelStr, ylabelStr)
    set(ax, 'Color', [0.15 0.15 0.15], 'XColor', 'white', 'YColor', 'white');
    hold(ax, 'on'); grid(ax, 'on');
    xlabel(ax, xlabelStr, 'Color', 'white', 'FontSize', 9);
    ylabel(ax, ylabelStr, 'Color', 'white', 'FontSize', 9);
    xlim(ax, [-60 20]);
    ylim(ax, [0 30]);
    title(ax, titleStr, 'Color', 'white', 'FontSize', 10);
end