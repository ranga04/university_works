clear all;
clc;
close all;

% Read the static image
staticImage = imread('red_square_static.jpg');
gingerbreadImage1 = imread('GingerBreadMan_first.jpg');
gingerbreadImage2 = imread('GingerBreadMan_second.jpg');

% Convert the images to grayscale
staticImageGray = rgb2gray(staticImage);
gingerbreadImageGray1 = rgb2gray(gingerbreadImage1);
gingerbreadImageGray2 = rgb2gray(gingerbreadImage2);

%Corner detection method to find the corners of the red square
cornersStatic = detectHarrisFeatures(staticImageGray);
cornersGingerbread1 = detectHarrisFeatures(gingerbreadImageGray1);
cornersGingerbread2 = detectHarrisFeatures(gingerbreadImageGray2);

% Visualize the detected corners
figure;
imshow(staticImage);
hold on;
plot(cornersStatic.selectStrongest(50));
title('Red Square Static Corners');

xlim([312.5 390.0])
ylim([233.3 297.5])

figure;
imshow(gingerbreadImage1);
hold on;
plot(cornersGingerbread1.selectStrongest(200));
title('First Img Corners');

figure;
imshow(gingerbreadImage2);
hold on;
plot(cornersGingerbread2.selectStrongest(200));
title('Second Img Corners');






% Creating optical flow object for estimating the optical flow using Lucas-Kanade method
opticFlow = opticalFlowLK('NoiseThreshold', 0.009);

% Estimating the optical flow from the first image to the second image
flow = estimateFlow(opticFlow, gingerbreadImageGray1);

% Estimating the optical flow for the second image
flow2 = estimateFlow(opticFlow, gingerbreadImageGray2);

figure;
imshow(gingerbreadImage1);
title('First Image');
hold on;
plot(flow, 'DecimationFactor', [25 25], 'ScaleFactor', 50, 'Color', 'b'); 
hold off;


figure;
imshow(gingerbreadImage2);
title('Second Image');
hold on;
plot(flow2, 'DecimationFactor', [25 25], 'ScaleFactor', 50, 'Color', 'b'); 
hold off;


% VideoReader and Optical Flow Initialization
vidReader = VideoReader('red_square_video.mp4');
opticFlow = opticalFlowLK('NoiseThreshold',0.009);

% Load Ground Truth Data
load('red_square_gt.mat');

% Read the First Frame and Initialize Tracking
frameRGB = readFrame(vidReader);
frameGray = rgb2gray(frameRGB);
corners = detectHarrisFeatures(frameGray);
position = corners.selectStrongest(1).Location;
trajectory = position;

% Variables for RMSE Calculation
numFrames = 0;
RMSE_x = [];
RMSE_y = [];
RMSE_combined = [];
estimated_positions = [];

% Frames Processing
while hasFrame(vidReader)
    numFrames = numFrames + 1;
    frameRGB = readFrame(vidReader);
    frameGray = rgb2gray(frameRGB);
    corners = detectHarrisFeatures(frameGray);
    distances = sqrt(sum((corners.Location - position).^2, 2));
    [minDist, idx] = min(distances);
    nearestCorner = corners.Location(idx, :);
    flow = estimateFlow(opticFlow, frameGray);
    position = nearestCorner + [flow.Vx(round(nearestCorner(2)), round(nearestCorner(1))), ...
                                flow.Vy(round(nearestCorner(2)), round(nearestCorner(1)))];
    trajectory = [trajectory; position];
    estimated_positions(numFrames, :) = position;
end
% Compute and Display RMSE for each frame
RMSE_values = zeros(numFrames, 1);
for i = 1:numFrames
    estimated_position = estimated_positions(i, :);
    if i <= size(gt_track_spatial, 1)
        ground_truth_position = gt_track_spatial(i, :);
        error_x = (estimated_position(1) - ground_truth_position(1))^2;
        error_y = (estimated_position(2) - ground_truth_position(2))^2;
        RMSE_x(i) = sqrt(error_x);
        RMSE_y(i) = sqrt(error_y);
        RMSE_combined(i) = sqrt(error_x + error_y);
        RMSE_values(i) = RMSE_combined(i);
        fprintf('RMSE for frame %d: %.4f\n', i, RMSE_values(i));
    else
        fprintf('No ground truth data available for frame %d\n', i);
    end
end

if numFrames > 0
    average_RMSE = mean(RMSE_values);
    fprintf('Average RMSE over all frames: %.4f\n', average_RMSE);
else
    fprintf('No frames processed.\n');
end


figure;
imshow(frameRGB);
hold on;
plot(trajectory(:,1), trajectory(:,2), 'b-', "LineWidth",3);
title('Estimated Trajectory')
xlim([250 387]);
ylim([197 324]);

figure;
imshow(frameRGB);
hold on;
plot(trajectory(:,1), trajectory(:,2), 'b-', 'LineWidth', 3);
plot(gt_track_spatial(:,1), gt_track_spatial(:,2), 'g-', 'LineWidth', 2);
legend('Estimated Trajectory', 'Ground Truth');
title('Estimated Trajectory vs. Ground Truth');
xlim([250 387]);
ylim([197 324]);

figure;
subplot(3, 1, 1); % RMSE X
plot(RMSE_x, 'r', 'LineWidth', 2);
title('RMSE X over Frames');
ylabel('RMSE');
xlabel('Frame Number');

subplot(3, 1, 2); % RMSE Y
plot(RMSE_y, 'g', 'LineWidth', 2);
title('RMSE Y over Frames');
ylabel('RMSE');
xlabel('Frame Number');

subplot(3, 1, 3); % RMSE Combined
plot(RMSE_combined, 'b', 'LineWidth', 2);
title('Combined RMSE over Frames');
ylabel('RMSE');
xlabel('Frame Number');

sgtitle('RMSE Analysis');