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

%% Create the IBVS object
VS = IBVSController();

while true
    % receive depth data
    lidarMsg = receive(lidarSub);
    % find depth
    [minDistance, index, Z] = VS.lidarDepth(lidarMsg);
    fprintf('Minimum Distance: %.3f\nAngle: %d\nDepth: %.3f\n', minDistance,index-1,Z);

    % receive image data
    imgMsg = receive(imgSub);
    featureImage = readImage(imgMsg);
    % Compute camera velocities using IBVS
    Vc = VS.computeCameraVelocity(featureImage,Z);
    fprintf('vx: %.3f\nvz: %.3f\n', Vc(1,1), Vc(2,1));

    % Control Turtlebot3 using velocities
    % ...
    
    pause(0.1);
end

% Delete the IBVS object
clear VS;
