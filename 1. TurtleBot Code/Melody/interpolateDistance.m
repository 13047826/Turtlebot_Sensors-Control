function d = interpolateDistance(corners)
% -- Set up -- %

% Target at further distance
% d1 = distance
% L1 = edgeLength

% Taget at closer distance
% d2 = distance
% L2 = edgeLength

% Load targets
load("RealTargetCorners.mat", 'targets');
%load("SimulatedTargetCorners.mat", 'targets');

% Compute bottom edge length
L = corners(2,2) - corners(1,2);

% Compare bottom edge length with available bottom edge lengths and note
% the index (distance

for k = 5:5:90
    if L > targets(k,5)
        break
    end
end

d1 = targets(k,3);
L1 = targets(k,5);
d2 = targets(k-5,3);
L2 = targets(k-5,5);

% Interpolate

d = d1 + (L-L1) * (d2-d1) / (L2 - L1);

end