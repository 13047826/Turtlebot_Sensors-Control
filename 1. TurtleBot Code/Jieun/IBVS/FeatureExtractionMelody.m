classdef FeatureExtractionMelody
    %FEATUREEXTRACTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
    end
    
    methods
        function obj = FeatureExtractionMelody()
            % Creates object            
        end

        function [centre,corners] = fastCentreCornersDetection(rgbImg)

            % Centre = x,y
            % Corners = x,y
            % This function gets sees blue and converts the image to greyscale with respect to blue
            close all
            
            % Breaking rgbImg into colour components
            r1 = rgbImg(:,:,1);
            g1 = rgbImg(:,:,2);
            b1 = rgbImg(:,:,3);
            
            justBlue = b1 - r1/2 - g1/2; 
            bw = justBlue > 40; % Threshold that turns the image to a binary image
            
            tempCorners = detectHarrisFeatures(bw);
            %tempCorners = detectFASTFeatures(bw); % Detected no corners for bw
            
            % Get corners
            corners = [min(tempCorners.Location(:,1)) min(tempCorners.Location(:,2));
                          min(tempCorners.Location(:,1)) max(tempCorners.Location(:,2)); ...
                          max(tempCorners.Location(:,1)) min(tempCorners.Location(:,2)); ...
                          max(tempCorners.Location(:,1)) max(tempCorners.Location(:,2))];   
            
            centre = [mean(corners(:,1)) mean(corners(:,2))];
            
            
            % Plot Debug
            imshow(bw);
            hold on
            plot(corners(:,1), corners(:,2), '.')
            plot(centre(:,1), centre(:,2), '.')
            
        end
    end
end

