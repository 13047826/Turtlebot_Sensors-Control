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

            % Sometimes boundary variable is empty and error pops up.
            % therefore check if boundary is not empty
            if isempty(boundary)
                m = [229  52;    % make it same as desiredFeaturePoints (0 error)
                     229 240;    
                     418  52;    
                     418 240];
                return; 
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

        function [xm, ym] = blueMarkerCentre(obj,img)
            r = img(:,:,1);
            g = img(:,:,2);
            b = img(:,:,3);
            % calculate blue
            blue = b - r/2 - g/2;
            bw = blue > 30;
            % Find the center of the image
            [x, y] = find(bw);
            xm = [];
            ym = [];
            if ~isempty(x) && ~isempty(y)
                xm = round(mean(x));
                ym = round(mean(y));
            end
        end
    end
end

