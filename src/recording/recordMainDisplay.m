function recordMainDisplay(display, videoFolder, videoFrameRate, numSteps)
% Separate function to record the main display

    fprintf('\n=== Recording Main Display ===\n');
    
    if ~isprop(display, 'Figure') || ~ishandle(display.Figure)
        fprintf('Main display figure not available\n');
        return;
    end
    
    fig = display.Figure;
    
    % Ensure figure is visible
    set(fig, 'Visible', 'on');
    drawnow;
    
    % Create video writer
    videoPath = fullfile(videoFolder, 'main_display_view.avi');
    writer = VideoWriter(videoPath);
    writer.FrameRate = videoFrameRate;
    open(writer);
    
    % Capture frames by stepping through the scenario again
    % This requires reloading the data
    load('MultiRadarGridBasedTrackingData.mat', 'radarPointCloudLog', 'egoPoseLog');
    
    % Recreate scenario for display
    [scenario, egoVehicle, radars] = helperCreateGridBasedRadarDrivingScenario();
    restart(scenario);
    
    fprintf('Capturing %d frames for main display...\n', numSteps);
    
    for step = 1:numSteps
        % Advance scenario
        while advance(scenario) && scenario.SimulationTime < step*0.1
            % Advance to correct time
        end
        
        % Get data
        measurements = radarPointCloudLog{step};
        egoPose = egoPoseLog{step};
        
        % Update display
        try
            display.step(scenario, egoVehicle, radars, measurements, [], []);
            
            % Capture frame
            drawnow;
            frame = getframe(fig);
            
            % Ensure dimensions are even
            frameSize = size(frame.cdata);
            if length(frameSize) >= 3
                if mod(frameSize(2), 2) ~= 0 || mod(frameSize(1), 2) ~= 0
                    newWidth = frameSize(2) - mod(frameSize(2), 2);
                    newHeight = frameSize(1) - mod(frameSize(1), 2);
                    frame.cdata = imresize(frame.cdata, [newHeight, newWidth]);
                end
                writeVideo(writer, frame);
            end
            
            if mod(step, 10) == 0
                fprintf('  Captured frame %d/%d\n', step, numSteps);
            end
        catch ME
            fprintf('Error at step %d: %s\n', step, ME.message);
        end
    end
    
    close(writer);
    fprintf('Main display video saved: %s (%d frames)\n', videoPath, numSteps);
end