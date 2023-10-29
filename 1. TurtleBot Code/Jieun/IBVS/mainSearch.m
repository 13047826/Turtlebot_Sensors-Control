%% Initialisation
clear all;
close all;
% Shutdown any existing ROS connection
rosshutdown;
% Create ROS Master
rosinit;
%% Publish and Subscribe
velPub = rospublisher('/Follower/cmd_vel','DataFormat','struct');
velMsg = rosmessage(velPub);
odomSub = rossubscriber('/Follower/odom','DataFormat','struct');
% wait and check to see if we receive msg
odomMsg = receive(odomSub,3)
lidarSub = rossubscriber('/Follower/scan','DataFormat','struct');
% wait and check to see if we receive msg
lidarMsg = receive(lidarSub,3)
imgSub = rossubscriber('/Follower/camera/rgb/image_raw');
% wait and check to see if we receive msg
imgMsg = receive(imgSub,3)
readRGBImage = readImage(imgMsg);
%% Figure Display Setup
rgbFig = figure;
imgSize = size(readRGBImage);
rgbImg = imshow(zeros(imgSize(1),imgSize(2),3,'uint8'));
displayText = text(10,25,'','Color','black','FontSize',16);
%% Create the IBVS object
C = Controller();
FE = FeatureExtraction();
% FE = FeatureExtractionMelody();

% Choose controlMode
C.setControlMode('IBVS');
% Starting State
state = 'SEARCH MODE';
% Ensure velocity starts at zero
velMsg.Linear.X = 0;
velMsg.Angular.Z = 0;

while true
    % Receive image data
    imgMsg = receive(imgSub);
    featureImage = readImage(imgMsg);
    
    % Try to find blue marker
    [xm, ym] = FE.blueMarkerCentre(featureImage);

    switch state
        case 'SEARCH MODE'
            if ~isempty(xm) && ~isempty(ym)
                % Blue marker is visible
                state = 'CONTROL MODE';
                fprintf('Switched to CONTROL MODE\n');
            else
                % No blue marker detected, stop all motion
                velMsg.Linear.X = 0;
                velMsg.Angular.Z = 0;
                send(velPub, velMsg);
                fprintf('STOPPED. WAITING FOR MARKER\n');
                continue;
            end

        case 'CONTROL MODE'

            % Update image display
            set(rgbImg,'CData',featureImage);
        
            % Compute measured feature points
            m = FE.extractFeatures(featureImage);
            % [centre,m] = fastCentreCornersDetection(rgbImg);
        
            % receive depth data
            lidarMsg = receive(lidarSub);
            % find depth
            [minDistance, index, Z, isValid] = C.lidarDepth(lidarMsg);
            if ~isValid
                fprintf('Invalid depth data. Skipping one iteration.\n');
                continue; % Skip to the next loop iteration
            end
            fprintf('Minimum Distance: %.3f\nAngle: %d\nDepth: %.3f\n', minDistance,index-1,Z);
            modeString = sprintf('Mode: %s | minDistance: %.3f | Angle: %d\n', C.getControlMode(), minDistance, index-1);
            set(displayText,'String', modeString); % Update text
            % Compute camera velocities using selected controlMode
            [vLinear, vAngular] = C.computeVelocity(m,Z);
            
            % Send velocity to Turtlebot
            velMsg.Linear.X = vLinear;
            velMsg.Angular.Z = vAngular;
            send(velPub,velMsg);
            fprintf('AngularVelocity: %.3f\n', vAngular);
    end
    drawnow;
    pause(0.1);
end

% Delete the IBVS object
clear C;
