clear all;
clc;
close all;


% read the video
source = VideoReader('car-tracking.mp4');  

% create and open the object to write the results
output = VideoWriter('frame_difference_output.mp4', 'MPEG-4');
open(output);

% vary the detection threshold
for thresh = 10:10:50     

    % read the first frame of the video as a background model
    bg = readFrame(source);
    bg_bw = rgb2gray(bg);           % convert background to greyscale

    % --------------------- process frames -----------------------------------
    % loop all the frames
    while hasFrame(source)
        fr = readFrame(source);     % read in frame
        fr_bw = rgb2gray(fr);       % convert frame to grayscale
        fr_diff = abs(double(fr_bw) - double(bg_bw));  % cast operands as double to avoid negative overflow

        % if fr_diff > thresh pixel in foreground
        fg = uint8(zeros(size(bg_bw)));
        fg(fr_diff > thresh) = 255;

        % update the background model
        bg_bw = fr_bw;

        % visualise the results
        figure(1),subplot(3,1,1), imshow(fr)
        title(['Threshold: ', num2str(thresh)])
        subplot(3,1,2), imshow(fr_bw)
        subplot(3,1,3), imshow(fg)
        drawnow

        writeVideo(output, fg);           % save frame into the output video
    end

    % reset the video reader to the beginning
    source.CurrentTime = 0;

end

close(output); % save video
