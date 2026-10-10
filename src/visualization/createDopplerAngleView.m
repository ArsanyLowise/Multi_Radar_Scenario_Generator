function [dopplerFig, rawScatter, compScatter, ax1, ax2] = createDopplerAngleView()
    dopplerFig = figure('Name', 'Doppler-Angle Comparison', ...
        'Position', [100, 100, 1400, 700], 'Color', [0.1 0.1 0.1]);

    % Left subplot - Raw Doppler
    ax1 = subplot(1,2,1);
    setupDopplerAxes(ax1, 'Raw Doppler', 'Azimuth Angle (degrees)', 'Raw Radial Velocity (m/s)');
    
    % Right subplot - Compensated Doppler
    ax2 = subplot(1,2,2);
    setupDopplerAxes(ax2, 'Ego-Compensated Doppler (Dynamic Only)', ...
        'Azimuth Angle (degrees)', 'Compensated Radial Velocity (m/s)');

    % Scatter plots
    rawScatter = scatter(ax1, [], [], 30, 'filled', 'MarkerFaceAlpha', 0.7);
    compScatter = scatter(ax2, [], [], 30, 'filled', 'MarkerFaceAlpha', 0.7);

    % Colorbars
    cb1 = colorbar(ax1);
    cb1.Color = 'white';
    cb1.Label.String = 'Range (m)';
    cb1.Label.Color = 'white';

    cb2 = colorbar(ax2);
    cb2.Color = 'white';
    cb2.Label.String = 'Range (m)';
    cb2.Label.Color = 'white';
end

function setupDopplerAxes(ax, titleStr, xlabelStr, ylabelStr)
    set(ax, 'Color', [0.15 0.15 0.15], 'XColor', 'white', 'YColor', 'white', ...
        'GridColor', [0.4 0.4 0.4], 'GridAlpha', 0.3, 'Box', 'on');
    hold(ax, 'on'); grid(ax, 'on');
    xlabel(ax, xlabelStr, 'Color', 'white', 'FontSize', 11);
    ylabel(ax, ylabelStr, 'Color', 'white', 'FontSize', 11);
    title(ax, titleStr, 'Color', 'white', 'FontSize', 12);
    xlim(ax, [-90 90]);
    ylim(ax, [-30 30]);
    
    % Add reference lines
    plot(ax, xlim, [0 0], 'w--', 'LineWidth', 1, 'Color', [0.8 0.8 0.8]);
    plot(ax, [0 0], ylim, 'w--', 'LineWidth', 1, 'Color', [0.8 0.8 0.8]);
    plot(ax, [-45 -45], ylim, 'r:', 'LineWidth', 1);
    plot(ax, [45 45], ylim, 'r:', 'LineWidth', 1);
end