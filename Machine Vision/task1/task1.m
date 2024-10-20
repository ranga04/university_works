clear all;
clc;
 close all;

% Reading the image
img = imread('Dog.jpg');

% Displaying the original image
figure, imshow(img), title('Original Image');

% Calculating the histograms
red = img(:,:,1);
green = img(:,:,2);
blue = img(:,:,3);

% Displaying the histograms
figure, subplot(3,1,1), imhist(red), title('Red Histogram');
subplot(3,1,2), imhist(green), title('Green Histogram');
subplot(3,1,3), imhist(blue), title('Blue Histogram');


% Convert the image to grayscale
gray = rgb2gray(img);

% Define a range of threshold values
thresh_values = 0.4;

% Define the types of noise
noise_types = {'Gaussian', 'Salt-and-Pepper', 'Electrical Interference'};

% Define the edge detection methods
edge_methods = {'Sobel', 'Prewitt', 'Canny'};

figure;

% Initialize separate figures for each algorithm
for noise_idx = 1:length(noise_types)
    % Applying Gaussian noise
    if noise_idx == 1
        noisyImg = imnoise(gray, 'gaussian', 0, 0.01);
    % Applying salt-and-pepper noise
    elseif noise_idx == 2
        noisyImg = imnoise(gray, 'salt & pepper', 0.02);
    % Applying electrical interference noise
    else
        noisyImg = imnoise(gray, 'speckle', 0.02);
    end
    
    for thresh_idx = 1:length(thresh_values)
        for edge_idx = 1:length(edge_methods)
            % Applying edge detection with the current threshold value
            edges = edge(noisyImg, edge_methods{edge_idx}, thresh_values(thresh_idx));

            % Displaying edge detection result
            subplot(length(noise_types), length(edge_methods), (noise_idx-1)*length(edge_methods) + edge_idx);
            imshow(edges);
            title([edge_methods{edge_idx} '|' noise_types{noise_idx} '|' num2str(thresh_values(thresh_idx))]);
        end
    end
end