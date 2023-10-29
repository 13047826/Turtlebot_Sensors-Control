clc;
clear all;
close all;
clear classes;

%------------------%
%    User Setup    %
%------------------%

camResolution = [679,506]; % [3280, 2464]; % Pixels (Pi Cam) / Improved
p = camResolution / 2;
f = 3.04; % mm
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

RGBSub = rossubscriber("/Follower/camera/rgb/image_raw"); %
ScanSub = rossubscriber("/Follower/scan"); 
tfSub = rossubscriber("/Follower/joint_states");
cmdVelPub = rospublisher("/Follower/cmd_vel");
odomSu = rossubscriber("/Follower/odom");

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
set(cam, "mode", 1);
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


    while get(cam, "mode") == 1 % Search Mode
        searchMode(cam, con, L);
        
        
        rgbImg = insertText(get(cam, "rgbImg"),[0 0],"Searching",'FontSize', 18, 'BoxColor', 'yellow');
        corners = get(cam,'featureCorners');
        centre = get(cam,'featureCentre');
        imshow(rgbImg);
        hold on

        if(isnan(corners) == false)
            plot(corners(:,1), corners(:,2), 'm.')
            plot(centre(:,1), centre(:,2), 'c.')
        end

    end

    while get(cam, "mode") == 2 % Follow Mode
        rgbImg = insertText(get(cam, "rgbImg"),[0 0],"Following",'FontSize', 18, 'BoxColor', 'green');
        imshow(rgbImg);
        corners = get(cam,'featureCorners');
        centre = get(cam,'featureCentre');
        imshow(rgbImg);
        hold on

        % If corners aren't available at this time return to search mode 
        if(isnan(corners) == true)
            set(cam, "mode",1);
            break
        end

        if(isnan(corners) == false) % If corners are available
            plot(corners(:,1), corners(:,2), 'm.')
            plot(centre(:,1), centre(:,2), 'c.')
        end

        followMode(cam,con,L);
        % cla reset;

    end

end

disp("End Program")
