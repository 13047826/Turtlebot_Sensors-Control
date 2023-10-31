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
set(rgbFig, 'WindowStyle', 'docked');
%% Error Plot Initialization
% Create arrays to store error values over time
timePoints = [];
epZValues = [];
epXValues = [];

%% Create the IBVS object
VS = IBVSControllerKpTest();
FE = FeatureExtraction();
% FE = FeatureExtractionMelody();

startTime = tic;    % Start timer to record graph
while toc(startTime) < 15
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

    % Compute PID velocities
    [vLinear, vAngular, epZ, epX] = VS.PIDControl(m, Z);

    % convert to vAngular to rad/s
    absWz = abs(vAngular/Z);
    % Need vAngular to be opposite sign of to consider axis direction
    Wz = sign(-vAngular)*absWz;  % sign returns opp sign (0 returns 0)
    Wz = double(Wz);             % velMsg.Angular.Z needs a double input
    vLinear = double(vLinear);   % velMsg.Linear.X needs a double input

    % Control Turtlebot3 using velocities
    velMsg.Linear.X = vLinear;
    % Using index value to compute angle
    velMsg.Angular.Z = Wz;
    send(velPub,velMsg);
    fprintf('AngularVelocity: %.3f\n', Wz);

    % Store error values and corresponding time
    timePoints(end+1) = toc(startTime);
    epZValues(end+1) = epZ;
    epXValues(end+1) = epX;

    pause(0.1);
end

% After loop, plot and save figure
figure;
plot(timePoints, epZValues, 'r', 'DisplayName', 'KpLinear');
hold on;
plot(timePoints, epXValues, 'b', 'DisplayName', 'KpAngular');
xlabel('Time (s)');
ylabel('Error');
legend;
title('KpL = 0.0, KpA = 0.12');
grid on;

% Save the figure as .jpg
saveas(gcf, 'Kp12.jpg');

% Delete the IBVS object
clear VS;
