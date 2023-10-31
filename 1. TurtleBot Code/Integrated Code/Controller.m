classdef Controller < matlab.mixin.SetGet
    properties
%         desiredFeaturePoints = [229  52;    % Bottom left
%                                 229 240;    % Top left
%                                 418  52;    % Bottom right
%                                 418 240];   % Top left
        desiredFeaturePoints = [194 153;    % Bottom left
                                194 808;    % Top left
                                1065 153;    % Bottom right
                                1065 808];   % Top left
        f = 530;                            % focal length 3.04mm in pixels    
        p = [320 240];                      % principle point assuming centre of image
        cameraOffset = 0.123;               % offset between lidar & camera
        lambda = 0.1;                       % optimal lambda from tuning

        eiX = 0;                            % Integral error for vx
        eiZ = 0;                            % Integral error for vz
        epreviousX = 0;                     % previous error for vx
        epreviousZ = 0;                     % previous error for vz

        
        KpLinear = 0.65;                     % Linear PID gains from PID tuning
        KiLinear = 0.001;
        KdLinear = 0.0;
    
        KpAngular = 0.12;                    % Angular PID gains from PID tuning
        KiAngular = 0.001;
        KdAngular = 0.015;

        desiredZ = 0.14;                   % Z where desiredFeaturePoints was measured

        maxLinearVelocity = 0.26;           % Maximum linear velocity
        minLinearVelocity = -0.26;          % Minimum linear velocity
        maxAngularVelocity = 1.82;          % Maximum angular velocity
        minAngularVelocity = -1.82;         % Minimum angular velocity

        controlMode = 'PID';               % Two options: IBVS, PID

        cmdVelPub;                          % cmdVel Publisher
        velM;                               % vel
    end
    
    methods
        function obj = Controller()
        end
        
        function Vc = IBVS(obj, m, Z)
            
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

        function [vLinear, vAngular] = PIDControl(obj, m, Z)
            
            % To find 2D point (x,y) 
            xy = (obj.desiredFeaturePoints-obj.p)/obj.f;    % S*
            mxy = (m - obj.p)/obj.f;                        % S

            % To find error e(t)
            e_2 = mxy - xy;
            % To put x in first row, y in second
            e_2 = e_2';
            % use 'reshape' to put in single column
            e = reshape(e_2,[],1);
            
            % grab errors for x and z
            eX = e(1:2:end);
            eZ = Z - obj.desiredZ;

            % Linear PID Controller
            epZ = eZ;                   % Proportional = actual distance
            obj.eiZ = obj.eiZ + epZ;    % integral = sum of error
            edZ = epZ - obj.epreviousZ; % Derivative = change in error
            vLinear = obj.KpLinear*epZ + obj.KiLinear*obj.eiZ + obj.KdLinear*edZ;   % PID formula
            obj.epreviousZ = epZ;       % Update previous distance

            % Angular PID Controller
            epX = sum(eX);              % Proportional
            obj.eiX = obj.eiX + epX;    % integral = sum of error
            edX = epX - obj.epreviousX; % Derivative = change in error
            vAngular = obj.KpAngular*epX + obj.KiAngular*obj.eiX + obj.KdAngular*edX;   % PID formula
            obj.epreviousX = epX;       % Update previous

            % Apply min/max velocities
            vLinear = max(obj.minLinearVelocity, min(obj.maxLinearVelocity, vLinear));
            vAngular = max(obj.minAngularVelocity, min(obj.maxAngularVelocity, vAngular));

        end


        function [vLinear, vAngular] = computeVelocity(obj,m,Z)
            switch obj.controlMode
                case 'IBVS'
                    Vc = obj.IBVS(m,Z);                         % camera velocities vx,vz
                    absWz = abs(Vc(1,1)/Z);                     % convert vx to rad/s
                    Wz = sign(-Vc(1,1))*double(absWz);          % Need opp sign of vx to consider axis direction
                    vLinear = double(Vc(2,1));                  % velMsg.Angular.Z needs double type
                    vAngular = Wz;
                case 'PID'
                    [vLinear,vAngular] = obj.PIDControl(m,Z);   % velocities from PID controller
                    absWz = abs(vAngular/Z);                    % convert to rad/s
                    Wz = sign(-vAngular)*double(absWz);         % Need opp sign to consider axis direction
                    vLinear = double(vLinear);                  % velMsg.Angular.Z needs double type
                    vAngular = Wz;
                otherwise
                    error('Invalid control mode.');
            end
        end

        % set controlMode
        function setControlMode(obj,mode)
            if ismember(mode,{'PID', 'IBVS'})
                obj.controlMode = mode;
            else
                error('Invalid control mode. Choose PID/IBVS.');
            end
        end

        % get controlMode
        function mode = getControlMode(obj)
            mode = obj.controlMode;
        end

        function updateVelocityMatrix(camera)
            var = velocityMatrix(camera);
            set(camera, "velM", var);
        end

        % send linear and angular velocity to publisher
        function publishVelMatrix(obj, linear_x, angular_z)
            publisher = get(obj, "cmdVelPub");
            velocityMatrix = rosmessage(publisher);
            velocityMatrix.Linear.X = linear_x;
            velocityMatrix.Angular.Z = angular_z;
            send(publisher, velocityMatrix);
        end
        
        % stop the turtlebot
        function stop(obj)
            publisher = get(obj, "cmdVelPub");
            msg = rosmessage(publisher);
            msg.Angular.Z = 0;
            msg.Linear.X = 0;
            send(publisher,msg);
        end

    end

    methods (Access = private)

    end
end