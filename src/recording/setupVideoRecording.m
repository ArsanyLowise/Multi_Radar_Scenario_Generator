function videoWriters = setupVideoRecording(videoFolder, videoFrameRate, recordVideos, enableDBSCAN)
% Setup video writers for all visualizations
    videoWriters = struct();
    
    if ~recordVideos
        return;
    end
    
    % Create folder if needed
    if ~exist(videoFolder, 'dir')
        mkdir(videoFolder);
        fprintf('Created video folder: %s\n', videoFolder);
    end
    
    % Delete any existing video files to avoid conflicts
    if exist(fullfile(videoFolder, 'front_radar_view.avi'), 'file')
        delete(fullfile(videoFolder, 'front_radar_view.avi'));
    end
    if exist(fullfile(videoFolder, 'doppler_angle_view.avi'), 'file')
        delete(fullfile(videoFolder, 'doppler_angle_view.avi'));
    end
    if enableDBSCAN && exist(fullfile(videoFolder, 'dbscan_results.avi'), 'file')
        delete(fullfile(videoFolder, 'dbscan_results.avi'));
    end
    
    % Front Radar Video
    videoWriters.front = VideoWriter(fullfile(videoFolder, 'front_radar_view.avi'));
    videoWriters.front.FrameRate = videoFrameRate;
    open(videoWriters.front);
    fprintf('Recording front radar view to: %s\n', fullfile(videoFolder, 'front_radar_view.avi'));
    
    % Doppler-Angle Video
    videoWriters.doppler = VideoWriter(fullfile(videoFolder, 'doppler_angle_view.avi'));
    videoWriters.doppler.FrameRate = videoFrameRate;
    open(videoWriters.doppler);
    fprintf('Recording Doppler-angle view to: %s\n', fullfile(videoFolder, 'doppler_angle_view.avi'));
    
    % DBSCAN Video
    if enableDBSCAN
        videoWriters.dbscan = VideoWriter(fullfile(videoFolder, 'dbscan_results.avi'));
        videoWriters.dbscan.FrameRate = videoFrameRate;
        open(videoWriters.dbscan);
        fprintf('Recording DBSCAN results to: %s\n', fullfile(videoFolder, 'dbscan_results.avi'));
    end
    
    % Give a moment for files to initialize
    pause(0.5);
end


