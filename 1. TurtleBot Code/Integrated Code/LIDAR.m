classdef LIDAR < matlab.mixin.SetGet

    properties
    scanSub % laser scan subscriber
    end

    methods
        function obj = LIDAR()

        end

        function [minDistance, index, Z, isValid] = lidarDepth(obj)
            lidarMsg = get(receive(obj.scanSub));
            % Record minimum distance and corresponding index
            [minDistance, index] = min(lidarMsg.Ranges);
            % Calculate angle of the minimum distance using index
            angleMinDistance = lidarMsg.AngleIncrement * (index-1);
            % Calculate cartesian x at this minimum distance
            cartesianX = minDistance*cos(angleMinDistance);
            % Subtract cameraOffset to calculate camera depth
            Z = cartesianX - obj.cameraOffset;
            
            isValid = true; % variable to check valid data
            % Validate the depth data
            if isnan(Z) || isinf(Z) || Z < 0
                isValid = false;
                return;
            end
        end
    end
end
