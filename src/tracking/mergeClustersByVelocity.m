function IDX = mergeClustersByVelocity(X, IDX, velocityThreshold)
    % Merge clusters that have similar average velocities
    
    if isempty(IDX)
        return;
    end
    
    uniqueClusters = unique(IDX);
    uniqueClusters = uniqueClusters(uniqueClusters > 0); % Exclude noise
    
    if length(uniqueClusters) <= 1
        return; % No merging needed
    end
    
    % Calculate mean velocity for each cluster
    clusterVelocities = zeros(length(uniqueClusters), 1);
    clusterSizes = zeros(length(uniqueClusters), 1);
    
    for i = 1:length(uniqueClusters)
        clusterIdx = uniqueClusters(i);
        clusterPoints = X(IDX == clusterIdx, 1); % Velocity values (FIRST column - your original)
        clusterVelocities(i) = mean(clusterPoints);
        clusterSizes(i) = sum(IDX == clusterIdx);
    end
    
    % Sort clusters by size (largest first) to keep larger cluster IDs
    [~, sortIdx] = sort(clusterSizes, 'descend');
    uniqueClusters = uniqueClusters(sortIdx);
    clusterVelocities = clusterVelocities(sortIdx);
    clusterSizes = clusterSizes(sortIdx);
    
    % Merge clusters with similar velocities
    merged = false(length(uniqueClusters), 1);
    
    for i = 1:length(uniqueClusters)
        if merged(i)
            continue;
        end
        
        currentCluster = uniqueClusters(i);
        currentVel = clusterVelocities(i);
        
        % Find clusters with similar velocities
        for j = i+1:length(uniqueClusters)
            if ~merged(j) && abs(clusterVelocities(j) - currentVel) <= velocityThreshold
                % Merge cluster j into cluster i
                IDX(IDX == uniqueClusters(j)) = currentCluster;
                merged(j) = true;
            end
        end
        merged(i) = true;
    end
    
    % Renumber clusters sequentially
    remainingClusters = unique(IDX(IDX > 0));
    newID = 1;
    for oldID = remainingClusters'
        IDX(IDX == oldID) = newID;
        newID = newID + 1;
    end
end