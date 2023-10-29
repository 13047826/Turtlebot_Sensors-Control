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

% m = [273 189;
%      273 240;
%      316 189;
%      316 240];      % Measured from image test_2.jpg

m = [369 190;
     369 239;
     384 190;
     384 239];      % Measured from image test_3.jpg

% m = [167 189;
%      167 240;
%      218 189;
%      218 240];      % Measured from image test_y.jpg

% m = [883 385;    % using 1920x1080 res
%      883 537;    
%      1039 385;    
%      1039 537];  

imshow('test_3.jpg');
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
%% To compute Lx for all four points
n = length(desiredFeaturePoints(:,1)); % number of points

Lx = [];
for i=1:n

    x = xy(i,1);
    y = xy(i,2);
    % Only need vx, wz (therefore two columns)
    Lxi = zeros(2,2);

    % vx component to compute rotation
    Lxi(1,1) = -1/Z;  % effect on x-coordinate
    Lxi(2,1) = 0;     % effect on y-coordinate
    % vz component to compute linear translation
    Lxi(1,2) = x/Z;     % effect on x-coordinate
    Lxi(2,2) = y/Z;    % effect on y-coordinate
    
    Lx = [Lx;Lxi];
end

%% Using Lx=Le to calculate pseudo-inverse of Le
Le = inv(Lx'*Lx)*Lx';
%% computing the camera velocity vc
Vc = -lambda*Le*e

% convert linear to angular velocity (w=v/r)
absWz = abs(Vc(1,1)/Z);
% Need Wz to be opposite sign of Vc to consider axis direction
if Vc(1,1)>0
    Wz = -absWz;
elseif Vc(1,1) < 0
    Wz = absWz;
else
    Wz = 0;
end
Wz