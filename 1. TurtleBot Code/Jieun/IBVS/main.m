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
modeText = text(10,25,'','Color','black','FontSize',16);
%% Create the IBVS object
C = Controller();
FE = FeatureExtraction();
% FE = FeatureExtractionMelody();

% Choose controlMode
C.setControlMode('IBVS');
state = 'SEARCH MODE';

while true
    % Receive image data
    imgMsg = receive(imgSub);
    featureImage = readImage(imgMsg);

    % Update image display
    set(rgbImg,'CData',featureImage);

    % Compute measured feature points
    m = FE.extractFeatures(featureImage);
    % [centre,m] = fastCentreCornersDetection(rgbImg);

    % receive depth data
    lidarMsg = receive(lidarSub);
    % find depth
    [minDistance, index, Z] = C.lidarDepth(lidarMsg);
    fprintf('Minimum Distance: %.3f\nAngle: %d\nDepth: %.3f\n', minDistance,index-1,Z);
    modeString = sprintf('Mode: %s | minDistance: %.3f | Angle: %d\n', C.getControlMode(), minDistance, index-1);
    set(modeText,'String', modeString); % Update text
    drawnow;
    % Compute camera velocities using selected controlMode
    [vLinear, vAngular] = C.computeVelocity(m,Z);
    
    % Send velocity to Turtlebot
    velMsg.Linear.X = vLinear;
    velMsg.Angular.Z = vAngular;
    send(velPub,velMsg);
    fprintf('AngularVelocity: %.3f\n', vAngular);
    pause(0.1);
end

% Delete the IBVS object
clear C;
