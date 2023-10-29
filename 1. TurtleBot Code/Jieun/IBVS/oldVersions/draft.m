clear all;
close all;

% parameters
p = [320 240];      % principle point assuming centre of image
f = 530;            % focal length computed from fov
R = eye(3);         % Camera Rotation
C = [0.073;
    -0.011; 
     0.084];        % Camera Translation (referred to xacro file)
desiredFeaturePoints = [229  52;    % Bottom left
                        229 240;    % Top left
                        418  52;    % Bottom right
                        418 240];   % Top left

% desiredFeaturePoints = [676   1;    % Using 1920.1070 res    
%                         676 542;    
%                         1246  1;    
%                         1246 542];    
  
% m = [295 189;
%      295 240;
%      347 189;
%      347 240];      % Measured from image test_1.jpg

m = [273 189;
     273 240;
     316 189;
     316 240];      % Measured from image test_2.jpg

% m = [369 190;
%      369 239;
%      384 190;
%      384 239];      % Measured from image test_3.jpg

% m = [167 189;
%      167 240;
%      218 189;
%      218 240];      % Measured from image test_y.jpg

% m = [883 385;    % using 1920x1080 res
%      883 537;    
%      1039 385;    
%      1039 537];  

imshow('test_2.jpg');
hold on;
plot(desiredFeaturePoints(:,1),desiredFeaturePoints(:,2),'go');
hold on;
plot(m(:,1),m(:,2),'ro');

Z = 1;              % Depth in metres
lambda = 0.1;

%% To find 2D point (x,y) to get S* and S
xy = (desiredFeaturePoints-p)/f;   % S*
mxy = (m-p)/f;                     % S
%% To find error e(t)
e_2 = mxy - xy;
% To put x in first row, y in second
e_2 = e_2';
% use 'reshape' to put in single column
e = reshape(e_2,[],1);
%% PID Controller
% parameters
eiX = 0;                            % Integral error for vx
eiZ = 0;                            % Integral error for vz
epreviousX = 0;                     % previous error for vx
epreviousZ = 0;                     % previous error for vz

        
KpLinear = 1;                       % Linear PID gains
KiLinear = 0.01;
KdLinear = 0.5;
    
KpAngular = 1;                      % Angular PID gains
KiAngular = 0.03;
KdAngular = 0.05;

desiredZ = 0.123;                   % Z where desiredFeaturePoints was measured
% grab errors for x and z
eX = e(1:2:end);
eZ = Z - desiredZ;
% Linear PID Controller
epZ = eZ;                   % Proportional = actual distance
eiZ = eiZ + epZ;    % integral = sum of error
edZ = epZ - epreviousZ; % Derivative = change in error
vLinear = KpLinear*epZ + KiLinear*eiZ + KdLinear*edZ   % PID formula
epreviousZ = epZ;       % Update previous distance

            % Angular PID Controller
epX = sum(eX);              % Proportional
eiX = eiX + epX;    % integral = sum of error
edX = epX - epreviousX; % Derivative = change in error
vAngular = KpAngular*epX + KiAngular*eiX + KdAngular*edX   % PID formula
epreviousX = epX;       % Update previous
absWz = abs(vAngular/Z);
Wz = sign(-vAngular)*absWz;  % sign returns opp sign (0 returns 0)
Wz = double(Wz)             % velMsg.Angular.Z needs a double input