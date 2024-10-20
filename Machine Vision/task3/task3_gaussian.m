clear all;
clc;
close all;


% read the video
source = VideoReader('car-tracking.mp4');  

% create and open the object to write the results
output = VideoWriter('gmm_output.mp4', 'MPEG-4');
open(output);

% vary the number of Gaussian components
for n_gaussians = 1:5   

    % create foreground detector object
    n_frames = 10;   % a parameter to vary
    detector = vision.ForegroundDetector('NumTrainingFrames', n_frames, 'NumGaussians', n_gaussians);

    % --------------------- process frames -----------------------------------
    % loop all the frames
    while hasFrame(source)
        fr = readFrame(source);     % read in frame

        fgMask = step(detector, fr);    % compute the foreground mask by Gaussian mixture models

        % create frame with foreground detection
        fg = uint8(zeros(size(fr, 1), size(fr, 2)));
        fg(fgMask) = 255;

        % visualise the results
        figure(1),subplot(2,1,1), imshow(fr)
        title(['Number of Gaussians: ', num2str(n_gaussians)])
        subplot(2,1,2), imshow(fg)
        drawnow

        writeVideo(output, fg);           % save frame into the output video
    end

    % reset the video reader to the beginning
    source.CurrentTime = 0;

end

close(output); % save video
