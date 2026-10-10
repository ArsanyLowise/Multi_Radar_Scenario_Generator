function tracker = setupTracker(sensorConfigs)
% Setup tracker with given sensor configurations
    tracker = trackerGridRFS('SensorConfigurations', sensorConfigs,...
         'HasSensorConfigurationsInput', true);
    
    tracker.GridLength = 120;
    tracker.GridWidth = 120;
    tracker.GridResolution = 1.5;
    tracker.GridOriginInLocal = [-tracker.GridLength/2, -tracker.GridWidth/2];
    tracker.NumParticles = 2e5;
    tracker.NumBirthParticles = 2e4;
    tracker.VelocityLimits = [-15, 15; -15, 15];
    tracker.BirthProbability = 0.025;
    tracker.ProcessNoise = 5*eye(2);
    tracker.DeathRate = 1e-3;
    tracker.FreeSpaceDiscountFactor = 1e-2;
    tracker.AssignmentThreshold = 9;
    tracker.MinNumCellsPerCluster = 3;
    tracker.ClusteringThreshold = 4;
    tracker.ConfirmationThreshold = [4, 5];
    tracker.DeletionThreshold = [4, 4];
    tracker.UseGPU = false;
end