classdef IBVSControllerVS
    properties
        desiredFeaturePoints = [229  52;    % Bottom left
                                229 240;    % Top left
                                418  52;    % Bottom right
                                418 240];   % Top left
        f = 530;                            % focal length 3.04mm in pixels    
        p = [320 240];                      % principle point assuming centre of image
        cameraOffset = 0.123;               % offset between lidar & camera
        KCam;                               % camera intrinsic parameter matrix
        lambda = 0.4;                       % reduce error by 10% each timestamp
    end
    
    methods
        %function obj = IBVSController(controlGain)
        function obj = IBVSControllerVS()
            % Camera Intrinsic Parameter for square pixel (fx=fy)
            obj.KCam = [obj.f     0 obj.p(1);
                            0 obj.f obj.p(2);
                            0     0        1];
        end
        
        function Vc = computeCameraVelocity(obj, m, Z)
            
            % To find 2D point (x,y) 
            xy = (obj.desiredFeaturePoints-obj.p)/obj.f;    % S*
            mxy = (m - obj.p)/obj.f;                        % S

            % To find error e(t)
            e_2 = mxy - xy;
            % To put x in first row, y in second
            e_2 = e_2';
            % use 'reshape' to put in single column
            e = reshape(e_2,[],1);
            
            % Compute Image Jacobian (L)
            n = length(obj.desiredFeaturePoints(:,1)); % number of points
            Lx = [];
            for i=1:n

                x = xy(i,1);
                y = xy(i,2);
                % Only need vx, wy (therefore two columns)
                Lxi = zeros(2,2);
            
                % vx component to compute rotation
                Lxi(1,1) = -1/Z;  % effect on x-coordinate
                Lxi(2,1) = 0;     % effect on y-coordinate
                % vz component to compute linear translation
                Lxi(1,2) = x/Z;     % effect on x-coordinate
                Lxi(2,2) = y/Z;    % effect on y-coordinate
            
                Lx = [Lx;Lxi];
            end

            % pinv function does the same as inv(Lx'*Lx)*Lx' from tutorial
            Le = pinv(Lx);
            % Camera velocities
            Vc = -obj.lambda*Le*e;
            
        end

        function [minDistance, index, Z] = lidarDepth(obj, lidarMsg)
            % Record minimum distance and corresponding index
            [minDistance, index] = min(lidarMsg.Ranges);
            % Calculate angle of the minimum distance using index
            angleMinDistance = lidarMsg.AngleIncrement * (index-1);
            % Calculate cartesian x at this minimum distance
            cartesianX = minDistance*cos(angleMinDistance);
            % Subtract cameraOffset to calculate camera depth
            Z = cartesianX - obj.cameraOffset;

            % Validate the depth data
            if isnan(Z) || isinf(Z) || Z < 0
                error('Invalid depth data. Script execution will end.');
            end
        end

    end

    methods (Access = private)

    end
end
