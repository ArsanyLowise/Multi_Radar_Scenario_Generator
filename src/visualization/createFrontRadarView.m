function [frontAx, targetScatter, targetInfo] = createFrontRadarView(position)

% Set default position if not provided
if nargin < 1 || isempty(position)
    position = [1200, 100, 500, 600];
end

% Create figure
frontFig = figure('Name', 'Front Radar - Live Target View', ...
    'Position', position, ...
    'Color', [0.1 0.1 0.1], ...
    'NumberTitle', 'off');

% Create axes
frontAx = axes('Parent', frontFig, 'Color', [0.15 0.15 0.15], ...
    'XColor', [0.9 0.9 0.9], 'YColor', [0.9 0.9 0.9], ...
    'GridColor', [0.4 0.4 0.4], 'GridAlpha', 0.3, 'Box', 'on');
hold(frontAx, 'on'); grid(frontAx, 'on'); axis equal;
xlim(frontAx, [-10 10]); ylim(frontAx, [0 30]);
xlabel(frontAx, 'Lateral (m)', 'Color', 'white');
ylabel(frontAx, 'Forward (m)', 'Color', 'white');
title(frontAx, 'Front Radar - Target', 'Color', 'white');

% Plot ego
plot(frontAx, 0, 0, 'ws', 'MarkerSize', 12, ...
    'MarkerFaceColor', 'w', 'LineWidth', 2);
text(frontAx, 0, 1.5, 'EGO', 'Color', 'white', ...
    'HorizontalAlignment', 'center');

% Lane markers
laneWidth = 3.5;
line(frontAx, [-laneWidth/2, -laneWidth/2], [0, 30], ...
    'Color', 'cyan', 'LineWidth', 1.5);
line(frontAx, [laneWidth/2, laneWidth/2], [0, 30], ...
    'Color', [0.8 0.8 0.8], 'LineWidth', 1, 'LineStyle', '--');
line(frontAx, [0, 0], [0, 30], 'Color', [0.98 0.86 0.36], 'LineWidth', 1.5);

% Range rings
for r = [10, 20, 30]
    theta = linspace(-pi/2, pi/2, 100);
    x_arc = r * sin(theta); y_arc = r * cos(theta);
    valid = abs(x_arc) <= 10;
    plot(frontAx, x_arc(valid), y_arc(valid), ...
        'Color', [0.5 0.5 0.5], 'LineWidth', 0.5, 'LineStyle', ':');
end

% Scatter for target detections
targetScatter = scatter(frontAx, [], [], 40, 'red', 'filled', ...
    'MarkerEdgeColor', 'white');

% Text for target info
targetInfo = text(frontAx, 0, 0, '', 'Color', 'cyan', ...
    'FontSize', 9, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');

end