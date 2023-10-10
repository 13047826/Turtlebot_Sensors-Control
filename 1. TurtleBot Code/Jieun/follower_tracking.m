%% Initialisation
clear all;
close all;
% Shutdown any existing ROS connection
rosshutdown;
% Create ROS Master
rosinit;
%% Publish and Subscribe
velPub = rospublisher('/Follower/cmd_vel');
velMsg = rosmessage(velPub);
odomSub = rossubscriber('/Follower/odom');
odomMsg = receive(odomSub);
% Subscribe to colour image
rgbSub = rossubscriber('/camera/color/image_raw');
imMsg2 = receive(rgbSub);
readRGBImage = readImage(imMsg2);
%% Find image size
% Divide y-axis pixel size by 2 (Left & Right side of image)
imgSize = size(readRGBImage);
centre = imgSize(2)/2;
%% Track Blue Cylinder Marker
% Initialise Pose
velMsg.Linear.X = 0;
velMsg.Angular.Z = 0;
send(velPub,velMsg);
yaw = 0;    % Set initial yaw as zero
origin =0;  % origin yaw angle
kp = 0.5;   % kp = proportional gain (higher = stronger control action & faster response)
kp2 = 0.002;    % Additional kp2 variable required to control turn rate proportional to pixel size

% Figure setup
rgbFig = figure;
rgbImg = imshow(zeros(imgSize(1),imgSize(2),3,'uint8'));
set(rgbFig, 'WindowStyle', 'docked');

% initialising values
xx = [];
yy = [];

tic;
while toc<60
    % image from subscriber
    imMsg2 = receive(rgbSub);
    img = readImage(imMsg2);
    % see blueMarkerCentre.m for details on function
    [xm, ym] = blueMarkerCentre(img);
    if ~isempty(xm) && ~isempty(ym)
        % dot size set to 3
        xx = max(1, xm-3):min(xm+3, size(img, 1));
        yy = max(1, ym-3):min(ym+3, size(img, 2));
        bwbw = zeros(size(img), 'uint8');
        bwbw(xx, yy) = 255;

        % Find difference between centre of image & red dot
        % rotate in z-axis (yaw)
        velMsg.Angular.Z = kp2 * (centre - ym);
        disp(['centre = {', num2str(centre), '} red_y: {', num2str(ym), '}'])
        
        % Message to display on figure
        figureMsg = 'Marker Detected.';

    else
        % Update figure message
        figureMsg = 'Searching Mode';

        % Return to origin when not available
        % searchMode = true
        returnToOrigin(odomSub, velPub, origin, kp, rgbImg, rgbSub, figureMsg, true);
    end

    send(velPub, velMsg);

    % Display centre of blue marker with red dot
    imgWithRedDot = img;
    
    % only run this when xx & yy not empty
    % NOTE: needed this when leader robot is out of sight (prevent error)
    if ~isempty(xx) && ~isempty(yy)
        imgWithRedDot(xx, yy, 1) = imgWithRedDot(xx, yy, 1) + bwbw(xx, yy);
    end
    set(rgbImg,'CData',imgWithRedDot);
    drawnow;   % update display
     
    % Only update image if figure exists
    % Without this, I get object deletion error when closing figure
    updateFigureText(rgbImg, figureMsg);

end

% Update figure message
figureMsg = sprintf('Ending Operation. Returning to Origin = %f ', origin);

% return to origin (searchMode = false)
returnToOrigin(odomSub, velPub, origin, kp, rgbImg, rgbSub, figureMsg, false);
% Stop the turtlebot
velMsg.Linear.X = 0;
velMsg.Angular.Z = 0;
send(velPub, velMsg);

% End script
rosshutdown;