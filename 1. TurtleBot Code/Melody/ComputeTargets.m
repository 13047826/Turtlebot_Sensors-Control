% This Computation of Targets searches
% Stored in a matrix [x, y, Z_distance, edgeLength_y, edgeLength_x];

tic

targets = zeros(90,3);
arrayCounter = 1;

for k = 30:10:200
    file = "Target" + int2str(k) + ".jpg";
    rgbImg = imread(file);
    [centre, corners] = fastCentreCornersDetection(rgbImg);
    
    % Adding corner co-ordinates
    for j = 1:4
        targets(arrayCounter,1) = corners(j,1);
        targets(arrayCounter,2) = corners(j,2);
        targets(arrayCounter,3) = k;
        arrayCounter = arrayCounter + 1;
    end
    
    % Adding centre co-ordinates
    targets(arrayCounter,1) = centre(:,1);
    targets(arrayCounter,2) = centre(:,2);
    targets(arrayCounter,3) = k;
    arrayCounter = arrayCounter + 1;
    
end

    % Adding the distance between two points
for j = 5:5:90
    targets(j,4) = targets(j-2,1) - targets(j-4,1);
    targets(j,5) = targets(j-3,2) - targets(j-4,2);
end

toc

save('TargetCorners','targets');
