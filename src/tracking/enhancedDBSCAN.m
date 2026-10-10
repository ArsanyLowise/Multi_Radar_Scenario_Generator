function IDX = enhancedDBSCAN(X, epsilon, MinPts, velocityThreshold)
    % Enhanced DBSCAN with velocity-based cluster merging
    % X: [radial_velocity, range]  <-- NOTE: YOUR ORIGINAL ORDER
    
    n = size(X, 1);
    
    % Handle empty or small input
    if n == 0
        IDX = [];
        return;
    elseif n == 1
        IDX = 1; % Single point forms its own cluster
        return;
    end
    
    IDX = zeros(n, 1);
    isCore = false(n, 1);
    
    C = 0;
    
    % Use Euclidean distance
    D = pdist2(X, X);
    
    % Find core points
    for i = 1:n
        neighbors = find(D(i, :) <= epsilon);
        if length(neighbors) >= MinPts
            isCore(i) = true;
        end
    end
    
    % Expand clusters from core points
    for i = 1:n
        if IDX(i) == 0 && isCore(i)
            C = C + 1;
            IDX(i) = C;
            
            seeds = find(D(i, :) <= epsilon);
            seeds = seeds(seeds ~= i);
            
            k = 1;
            while k <= length(seeds)
                j = seeds(k);
                
                if IDX(j) == 0
                    IDX(j) = C;
                    
                    if isCore(j)
                        newNeighbors = find(D(j, :) <= epsilon);
                        newNeighbors = newNeighbors(IDX(newNeighbors) == 0);
                        seeds = [seeds, newNeighbors];
                    end
                end
                
                k = k + 1;
            end
        end
    end
    
    % Mark noise points
    IDX(IDX == 0) = -1;
    
    % POST-PROCESSING: Merge clusters with similar velocities
    if nargin >= 4 && ~isempty(velocityThreshold)
        IDX = mergeClustersByVelocity(X, IDX, velocityThreshold);
    end
end