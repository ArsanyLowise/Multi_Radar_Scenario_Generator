function compensatedRates = compensateEgoMotion(rangeRates, azimuths, egoSpeed)
% Compensate radar range rates for ego vehicle motion
    compensatedRates = zeros(size(rangeRates));
    for i = 1:length(rangeRates)
        vProj = egoSpeed * cosd(azimuths(i));
        compensatedRates(i) = rangeRates(i) + vProj;
    end
end