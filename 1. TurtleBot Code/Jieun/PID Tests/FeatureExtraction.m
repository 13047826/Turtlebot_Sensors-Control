classdef FeatureExtraction
    %FEATUREEXTRACTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        rgbFig = [];
        rgbImg = [];
    end
    
    methods
        function obj = FeatureExtraction()
            % Creates object            
        end

        function m = extractFeatures(obj,featureImage)
            % extract colours
            r = featureImage(:,:,1);
            g = featureImage(:,:,2);
            b = featureImage(:,:,3);
            
            % Calculate blue
            justBlue = b - r/2 - g/2;
            % Find boundaries of the blue marker
            bw = justBlue > 40; % Anything greater than this
            % bwboundaires function returns exterior boundaires of white regions
            % cell array type with each cell containing a set of pixel coordinates that
            % form a single boundary
            boundaries = bwboundaries(bw);
            % Find largest boundary (remove noise/other boundaires to detect blue
            % boundary only
            largestBoundary = 0;
            for k = 1:length(boundaries)
                % get size using number of elements for the boundaries
                currentSize = numel(boundaries{k});
                if currentSize > largestBoundary
                    % Update largest
                    largestBoundary = currentSize;
                end
            end
            
            % Extract the largest boundary
            boundary = [];
            for k = 1:length(boundaries)
                if numel(boundaries{k}) == largestBoundary
                    boundary = boundaries{k};
                    break;
                end
            end
            
            xMin = min(boundary(:,2));
            xMax = max(boundary(:,2));
            yMin = min(boundary(:,1));
            yMax = max(boundary(:,1));
            
            m = [xMin, yMin;    % Top-left
                 xMin, yMax;    % Bottom-left
                 xMax, yMin;    % Top-right
                 xMax, yMax];   % Bottom-right    % I followed the same order as Visual Servoing tutorial for easier implementation
        end
    end
end

