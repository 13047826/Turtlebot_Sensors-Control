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
% set(rgbFig, 'WindowStyle', 'docked');
%% Create the IBVS object
VS = IBVSControllerVS();
FE = FeatureExtraction();
% FE = FeatureExtractionMelody();

while true
    % Receive image data
    imgMsg = receive(imgSub);
    featureImage = readImage(imgMsg);

    % Update image display
    set(rgbImg,'CData',featureImage);
    drawnow;

    % Compute measured feature points
    m = FE.extractFeatures(featureImage);
    % [centre,m] = fastCentreCornersDetection(rgbImg);

    % receive depth data
    lidarMsg = receive(lidarSub);
    % find depth
    [minDistance, index, Z] = VS.lidarDepth(lidarMsg);
    fprintf('Minimum Distance: %.3f\nAngle: %d\nDepth: %.3f\n', minDistance,index-1,Z);

    % Compute camera velocities using IBVS
    Vc = VS.computeCameraVelocity(m,Z);
    % convert linear to angular velocity (w=v/r)
    absWz = abs(Vc(1,1)/Z);
    % Need Wz to be opposite sign of Vc to consider axis direction
    Wz = sign(-Vc(1,1))*absWz;  % sign returns opp sign (0 returns 0)
    Wz = double(Wz);            % velMsg.Angular.Z needs a double input
    % fprintf('vz: %.3f\nwz: %.3f\n', Vc(2,1), Wz);

    % Control Turtlebot3 using velocities
    velMsg.Linear.X = Vc(2,1);
    % Using index value to compute angle

    velMsg.Angular.Z = Wz;
    send(velPub,velMsg);
    fprintf('AngularVelocity: %.3f\n', Wz);
    pause(0.1);
end

% Delete the IBVS object
clear VS;
