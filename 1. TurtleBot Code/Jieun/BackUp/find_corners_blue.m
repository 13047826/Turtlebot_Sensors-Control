%% Resources
% https://au.mathworks.com/help/supportpkg/raspberrypiio/ref/track-a-green-ball.html
% https://au.mathworks.com/help/images/boundary-tracing-in-images.html
% https://au.mathworks.com/help/images/ref/bwboundaries.html
clear all;
close all;

%% Extract color components
img = imread('target_image.jpg');
image(img);
r = img(:,:,1);
g = img(:,:,2);
b = img(:,:,3);

%% Calculate blue
justBlue = b - r/2 - g/2;

%% Threshold the blue image
image(justBlue);

%% Find center 
[x, y] = find(justBlue);
if ~isempty(x) && ~isempty(y)
    xm = round(mean(x));
    ym = round(mean(y));
    xx = max(1, xm-3):min(xm+3, size(justBlue, 1));
    yy = max(1, ym-3):min(ym+3, size(justBlue, 2));
    bwbw = zeros(size(justBlue), 'uint8');
    bwbw(xx, yy) = 255;
    imgWithRedDot = img; % Create a copy of the original image
    imgWithRedDot(xx, yy, 1) = imgWithRedDot(xx, yy, 1) + bwbw(xx, yy);
    image(imgWithRedDot);
end

%% Find boundaries of the blue marker
bw = justBlue > 40; % Anything greater than this
% bwboundaires function returns exterior boundaires of white regions
% cell array type with each cell containing a set of pixel coordinates that
% form a single boundary
boundaries = bwboundaries(bw);
% Find largest boundary (remove noise/other boundaires to detect blue
% boundary only
largestBoundary = 0;
for k = 1:length(boundaries)
    % get size using number of elements for the boundaries
    currentSize = numel(boundaries{k});
    if currentSize > largestBoundary
        % Update largest
        largestBoundary = currentSize;
    end
end

% Extract the largest boundary
boundary = [];
for k = 1:length(boundaries)
    if numel(boundaries{k}) == largestBoundary
        boundary = boundaries{k};
        break;
    end
end

xMin = min(boundary(:,2));
xMax = max(boundary(:,2));
yMin = min(boundary(:,1));
yMax = max(boundary(:,1));

corners = [xMin, yMin;    % Top-left
           xMin, yMax;    % Bottom-left
           xMax, yMin;    % Top-right
           xMax, yMax];   % Bottom-right    % I followed the same order as Visual Servoing tutorial for easier implementation

%% Plotting the corners with red
imgWithRedCorners = imgWithRedDot;
for i = 1:4
    x = corners(i, 1);
    y = corners(i, 2);
    xx = max(1, x-3):min(x+3, size(justBlue, 2));
    yy = max(1, y-3):min(y+3, size(justBlue, 1));
    imgWithRedCorners(yy, xx, 1) = 255;
end

% Display the result with the red center dot and red corners
image(imgWithRedCorners);

% Create a scatter plot of the corners
figure;
scatter(corners(:,1), corners(:,2), 'ro'); % Plot corners as red circles
xlim([0, 640]); % Set x-axis limits
ylim([0, 480]); % Set y-axis limits
xlabel('X Coordinate');
ylabel('Y Coordinate');
title('Corner Visualisation');
% testing to see values with grid on
% grid on;
