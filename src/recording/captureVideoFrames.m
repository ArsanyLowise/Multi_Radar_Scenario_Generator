function captureVideoFrames(videoWriters, frontRadarAx, dopplerFig, dbscanFig, enableDBSCAN, step)
% Added step parameter for debugging
    
    % Force figure updates
    drawnow;
    
    % Front Radar Video
    if isfield(videoWriters, 'front') && isvalid(videoWriters.front)
        try
            % Make sure the figure is visible and active
            fig = frontRadarAx.Parent;
            
            % Ensure figure is visible
            if strcmp(fig.Visible, 'off')
                fig.Visible = 'on';
            end
            
            % Capture frame
            frontFrame = getframe(fig);
            
            % Check if frame is valid
            if ~isempty(frontFrame) && ~isempty(frontFrame.cdata)
                writeVideo(videoWriters.front, frontFrame);
                if step == 1 || mod(step, 10) == 0
                    fprintf('Step %d: Front frame captured, size: %dx%d\n', ...
                        step, size(frontFrame.cdata, 2), size(frontFrame.cdata, 1));
                end
            else
                fprintf('Warning: Empty front frame at step %d\n', step);
            end
        catch ME
            fprintf('Error capturing front frame at step %d: %s\n', step, ME.message);
        end
    end
    
    % Doppler Fig Video
    if isfield(videoWriters, 'doppler') && isvalid(videoWriters.doppler) && ishandle(dopplerFig)
        try
            % Ensure figure is visible
            if strcmp(dopplerFig.Visible, 'off')
                dopplerFig.Visible = 'on';
            end
            
            figure(dopplerFig);
            drawnow;
            
            dopplerFrame = getframe(dopplerFig);
            if ~isempty(dopplerFrame) && ~isempty(dopplerFrame.cdata)
                writeVideo(videoWriters.doppler, dopplerFrame);
                if step == 1 || mod(step, 10) == 0
                    fprintf('Step %d: Doppler frame captured\n', step);
                end
            end
        catch ME
            fprintf('Error capturing Doppler frame at step %d: %s\n', step, ME.message);
        end
    end
    
    % DBSCAN Video
    if enableDBSCAN && isfield(videoWriters, 'dbscan') && isvalid(videoWriters.dbscan) && ...
       exist('dbscanFig', 'var') && ishandle(dbscanFig)
        try
            % Ensure figure is visible
            if strcmp(dbscanFig.Visible, 'off')
                dbscanFig.Visible = 'on';
            end
            
            figure(dbscanFig);
            drawnow;
            
            dbscanFrame = getframe(dbscanFig);
            if ~isempty(dbscanFrame) && ~isempty(dbscanFrame.cdata)
                writeVideo(videoWriters.dbscan, dbscanFrame);
                if step == 1 || mod(step, 10) == 0
                    fprintf('Step %d: DBSCAN frame captured\n', step);
                end
            end
        catch ME
            fprintf('Error capturing DBSCAN frame at step %d: %s\n', step, ME.message);
        end
    end
end
