clc;
clear all;
close all;
clear classes;

%------------------%
%    User Setup    %
%------------------%

camResolution = [1280, 960]; % Simulation [679,506]; % Pi Cam [1280, 960]
p = camResolution / 2;
f = 1000; % mm
load('SimTargetCorners.mat', 'targets');
targetCorners = [229  52;    % Bottom left
                229 240;    % Top left
                418  52;    % Bottom right
                418 240];   % Top left
targetDistance = targets(4,5);

%------------------%
% Initialising ROS %
%------------------%
disp("Initialising ROS");

ipaddress = "192.168.1.31"; % Personal "192.168.1.31"; % Real turtlebot set up % 192.168.0.100
rosshutdown;
rosinit(ipaddress);

RGBSub = rossubscriber("/Follower/camera/rgb/image_raw"); %"/Follower/camera/rgb/image_raw" "/raspicam_node/image/compressed"
ScanSub = rossubscriber("/Follower/scan"); % "/Follower/scan" "/scan"
cmdVelPub = rospublisher("Follower/cmd_vel"); % "/cmd_vel"

%--------%
% Set up %
%--------%

% Robot Class?
% Set up mode*

% Setting up camera

cam = Camera; % Creating a camera object through handle 'cam'
set(cam, "resolution", camResolution);
set(cam, "rgbImgSub", RGBSub);

set(cam, "targetCorners", targetCorners);
set(cam, "mode", 2);
set(cam, "f", f);
set(cam, "p", p);

% Setting Up Controller
con = Controller;
set(con, "cmdVelPub", cmdVelPub);
velMsg = rosmessage(cmdVelPub); % Making sure turtlebot is not moving
velMsg.Angular.Z = 0;   
velMsg.Linear.X = 0;  
set(con, "velM", velMsg);
send(cmdVelPub,velMsg);

% Setting Up LIDAR;
L = LIDAR;
set(L, "scanSub", ScanSub);


%-----------%
% Main Loop %
%-----------%

disp("-----------------------------------")
disp("Follower Turtlebot is now searching")
disp("-----------------------------------")

while get(cam, "mode") > 0

    % ----------- %
    % Search Mode %
    % ----------- %

    while get(cam, "mode") == 1 % Search Mode
        con.stop;                                               % stop moving the turtlebot
        cam.updateRGBImg;
        cam.updateFeatureCentreCorners;
        rgbImg = insertText(get(cam, "rgbImg"),[0 0],"Searching", ...
            'FontSize', 18, 'BoxColor', 'yellow');
        imshow(rgbImg);
        hold on

        if(isnan(get(cam,'featureCorners')) == false)
            plot(corners(:,1), corners(:,2), 'mo')
            plot(centre(:,1), centre(:,2), 'ro')
        end

        if (isnan(get(cam, "featureCorners")) == false)
            con.stop; % stop moving the turtlebot
            set(cam, 'mode', 2);
            disp("-----------------------------------")
            disp("Follower Turtlebot is now following")
            disp("-----------------------------------")
        end

    end

    % ----------- %
    % Follow Mode %
    % ----------- %
    
    while get(cam, "mode") == 2 % Follow Mode
        
        cam.updateRGBImg;                                       % Update figure
        cam.updateFeatureCentreCorners;
        rgbImg = insertText(get(cam, "rgbImg"),[0 0], ...       
            "Following",'FontSize', 18, 'BoxColor', 'green');
        imshow(rgbImg);
        hold on
        
        if(isnan(get(cam,'featureCorners')) == true)            % Return to search mode if corners unavailable
            set(cam, "mode",1);
            break
        end

        if(isnan(get(cam,'featureCorners')) == false)           % Plot corners if available
            plot(corners(:,1), corners(:,2), 'mo')
            plot(centre(:,1), centre(:,2), 'ro')
        end

        
        if (isnan(get(cam, "featureCorners")) == false)         % If feature centre was detected - complete visual servoing
            
            Z = L.lidarDepth();                                 % Get distance to feature
            m = get(cam, "featureCorners");                     % Get observed corners
            [vLinear, vAngular] = con.computeVelocity(m,Z);     % Compute appropriate velocities
            con.publishVelMatrix(vLinear, vAngular);            % Publisher cmdvel 
           
        end

    end

end

disp("End Program")
