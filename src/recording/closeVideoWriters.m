function closeVideoWriters(videoWriters, enableDBSCAN, numSteps)
% Close video writers and show statistics
    persistent hasBeenClosed
    
    if isempty(hasBeenClosed)
        hasBeenClosed = false;
    end
    
    if hasBeenClosed
        fprintf('Note: Video writers already closed, skipping...\n');
        return;
    end
    
    fprintf('\n=== Closing Video Writers ===\n');
    
    if nargin < 3
        numSteps = []; % Not provided
    end
    
    if isfield(videoWriters, 'front') && isvalid(videoWriters.front)
        framesWritten = videoWriters.front.FrameCount;
        close(videoWriters.front);
        fprintf('Front radar: %d frames written\n', framesWritten);
        if framesWritten == 0
            fprintf('  WARNING: No frames written to front radar video!\n');
        end
    end
    
    if isfield(videoWriters, 'doppler') && isvalid(videoWriters.doppler)
        framesWritten = videoWriters.doppler.FrameCount;
        close(videoWriters.doppler);
        fprintf('Doppler view: %d frames written\n', framesWritten);
        if framesWritten == 0
            fprintf('  WARNING: No frames written to Doppler video!\n');
        end
    end
    
    if enableDBSCAN && isfield(videoWriters, 'dbscan') && isvalid(videoWriters.dbscan)
        framesWritten = videoWriters.dbscan.FrameCount;
        close(videoWriters.dbscan);
        fprintf('DBSCAN view: %d frames written\n', framesWritten);
        if framesWritten == 0
            fprintf('  WARNING: No frames written to DBSCAN video!\n');
        end
    end
    
    if ~isempty(numSteps)
        fprintf('Expected frames: ~%d\n', numSteps);
        if framesWritten < numSteps
            fprintf('  WARNING: Only captured %.1f%% of expected frames\n', ...
                100*framesWritten/numSteps);
        end
    end
    
    hasBeenClosed = true;
end




