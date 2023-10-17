classdef Testing_movement

    properties
        CamParam
        Threshold
        RGBCamSub
        DCamSub
        CmdVelPub
    end

    methods
        function obj = Testing_movement(camParam, threshold, ipaddress)
            obj.CamParam = camParam;
            obj.Threshold = threshold;

            % Initialise ROS
            rosshutdown;
            rosinit(ipaddress);

            % Create subscribers and publishers
            obj.RGBCamSub = rossubscriber("camera_rgb_topic"); 
            obj.DCamSub = rossubscriber("camera_depth_topic"); 
            obj.CmdVelPub = rospublisher("cmd_vel_topic"); 
        end

        % Search for leader
        function [state, leaderFound] = searchLeader(obj)
            % Capture Images
            rgbImgData = receive(obj.RGBCamSub, 5);
            dImgData = receive(obj.DCamSub, 5);

            rgbImg = readImage(rgbImgData);
            dImg = readImage(dImgData);

            % Detect leader
            [leaderFound, leaderPosition] = obj.detectLeader(rgbImg);

            if leaderFound
                % Rotate towards the leader
                obj.rotateTowardsLeader(leaderPosition);
                state = 2; % Transition to Follow Mode
            else
                state = 1; % Continue Searching
            end
        end

        % Movement of leader 
        function rotateTowardsLeader(obj, leaderPosition)
            % Define the desired rotation speed (angular velocity)
            rotationSpeed = 0.2; 

            % Get the leader's x and y position from the leaderPosition vector
            leaderX = leaderPosition(1);
            leaderY = leaderPosition(2);

            % Calculate the angle between the follower and the leader
            angleToLeader = atan2(leaderY, leaderX);

            % Calculate the desired angular velocity to rotate towards the leader
            angularVelocity = rotationSpeed * sign(angleToLeader);

            % Create a ROS message for velocity control
            velMsg = rosmessage(obj.CmdVelPub);
            velMsg.Angular.Z = angularVelocity;

            % Publish angular velocity to cmdVelPub
            send(obj.CmdVelPub, velMsg);
        end

        % Follow the leader
        function [state, leaderFound] = followLeader(obj)
        end
    end
end
