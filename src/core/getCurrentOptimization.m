function optimizationParams = getCurrentOptimization()
    % Default optimization parameters
    optimizationParams = struct();
    
    optimizationParams.numLanes = 2;
   
    % Road parameters
    optimizationParams.roadCurvature = -10;      % (+/-)
    optimizationParams.roadLength = 200;          
    optimizationParams.laneWidth = 3.5;
    optimizationParams.wayPointAmount = 50;      % For road curvature (capital P)
    
    % Barrier parameters
    optimizationParams.barrierDistance = 0;     
    optimizationParams.barrierHeight = 1.0;       
    optimizationParams.barrierWidth = 0.5; 
    
    % Ego vehicle parameters
    optimizationParams.egoSpeed = 30;              
    optimizationParams.egoAcceleration = 0.2;      
    optimizationParams.egoLength = 5;               
    optimizationParams.egoWidth = 2;                 
    optimizationParams.egoStartX = 20;              
    optimizationParams.egoLane = 2;                   % Lane number (1=left, 2=right)
    optimizationParams.waypointCountEgo = 5;          % For ego waypoints (lowercase w)
    
    % Target vehicle parameters
    optimizationParams.targetSpeed = 25;             
    optimizationParams.targetAcceleration = 0.9;     
    optimizationParams.targetLength = 4.5;           
    optimizationParams.targetWidth = 2;               
    optimizationParams.targetStartX = 150;            
    optimizationParams.targetLane = 1;                 % Lane number (1=left, 2=right)
    optimizationParams.targetDirection = -1;           % 1 for same direction, -1 for opposite
    optimizationParams.waypointAmount = 5;             % For target waypoints (lowercase w)
    
    % Scenario parameters
    optimizationParams.initialDistance = 20;          % Initial distance between vehicles (m)
    optimizationParams.sampleTime = 0.1;
    optimizationParams.simTime = 30; 
    
    % Trajectory parameters
    optimizationParams.smoothTrajectory = true;
end