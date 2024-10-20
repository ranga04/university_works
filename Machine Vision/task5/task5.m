clear all;
clc;
close all;


% Load and preprocess data
digitDatasetPath = fullfile(toolboxdir('nnet'), 'nndemos', 'nndatasets', 'DigitDataset');
imds = imageDatastore(digitDatasetPath, 'IncludeSubfolders', true, 'LabelSource', 'foldernames');
imds.ReadFcn = @(loc)imresize(imread(loc), [32, 32]);
[imdsTrain, imdsValidation] = splitEachLabel(imds, 0.7, 'randomized');

baselineLayers = [ 
    imageInputLayer([32 32 1],'Name','input') 
 
    convolution2dLayer(5,6,'Padding','same','Name','conv_1') 
    averagePooling2dLayer(2,'Stride',2,'Name','avgpool_1') 
 
    convolution2dLayer(5,16,'Padding','same','Name','conv_2') 
    averagePooling2dLayer(2,'Stride',2,'Name','avgpool_2') 
 
    fullyConnectedLayer(120,'Name','fc_1') 
    fullyConnectedLayer(84,'Name','fc_2') 
    fullyConnectedLayer(10,'Name','fc_3') 
    softmaxLayer('Name','softmax') 
    classificationLayer('Name','output')];
% Training options for the baseline model
baselineOptions = trainingOptions('sgdm', ... 
    'InitialLearnRate',0.0001, ... 
    'MaxEpochs',10, ... 
    'Shuffle','every-epoch', ... 
    'ValidationData',imdsValidation, ... 
    'ValidationFrequency',30, ... 
    'Verbose',false, ... 
    'Plots','training-progress'); 
% Train the baseline network
baselineNet = trainNetwork(imdsTrain, baselineLayers, baselineOptions);

% Classify validation images and compute accuracy for the baseline model
baselineYPred = classify(baselineNet, imdsValidation);
baselineYValidation = imdsValidation.Labels;
baselineAccuracy = sum(baselineYPred == baselineYValidation)/numel(baselineYValidation);
fprintf('Baseline Accuracy: %f\n', baselineAccuracy);

% improved model

%improved model layers
layers = [
    imageInputLayer([32 32 1],'Name','input')
    
    convolution2dLayer(5,6,'Padding','same','Name','conv_1')
    reluLayer('Name','relu_1')
    averagePooling2dLayer(2,'Stride',2,'Name','avgpool_1')
    
    convolution2dLayer(5,16,'Padding','same','Name','conv_2')
    reluLayer('Name','relu_2')
    averagePooling2dLayer(2,'Stride',2,'Name','avgpool_2')
    
    fullyConnectedLayer(120,'Name','fc_1')
    reluLayer('Name','relu_3')
    dropoutLayer(0.4, 'Name', 'dropout_1') 
    
    fullyConnectedLayer(84,'Name','fc_2')
    reluLayer('Name','relu_4')
    dropoutLayer(0.4, 'Name', 'dropout_2') 
    
    fullyConnectedLayer(10,'Name','fc_3')
    softmaxLayer('Name','softmax')
    classificationLayer('Name','output')];


% Define the improved model training options
options = trainingOptions('adam', ...
    'InitialLearnRate',0.001, ... 
    'MaxEpochs',20, ... 
    'Shuffle','every-epoch', ...
    'ValidationData',imdsValidation, ...
    'ValidationFrequency',30, ...
    'Verbose',true, ...
    'Plots','training-progress');

% Training the improved network
net = trainNetwork(imdsTrain,layers,options);

% Classify validation images and compute metrics for the improved model
YPred = classify(net, imdsValidation);
YValidation = imdsValidation.Labels;
improvedAccuracy = sum(YPred == YValidation) / numel(YValidation);
fprintf('Improved Model Accuracy: %f\n', improvedAccuracy);

% Comparison
fprintf('Improvement in accuracy: %f\n', improvedAccuracy - baselineAccuracy);


% Calculating the confusion matrix
confMatBaseline = confusionmat(baselineYValidation, baselineYPred);
confMatImproved = confusionmat(YValidation, YPred);

% Calculating Precision, Recall, and F1 Score for baseline
precisionBaseline = diag(confMatBaseline) ./ sum(confMatBaseline, 2);
recallBaseline = diag(confMatBaseline) ./ sum(confMatBaseline, 1)';
F1Baseline = 2 * (precisionBaseline .* recallBaseline) ./ (precisionBaseline + recallBaseline);

% Calculating Precision, Recall, and F1 Score for improved model
precisionImproved = diag(confMatImproved) ./ sum(confMatImproved, 2);
recallImproved = diag(confMatImproved) ./ sum(confMatImproved, 1)';
F1Improved = 2 * (precisionImproved .* recallImproved) ./ (precisionImproved + recallImproved);

% Displaying the results for both models
fprintf('\nBaseline Model Metrics:\n');
fprintf('Precision: %f\n', mean(precisionBaseline, 'omitnan'));
fprintf('Recall: %f\n', mean(recallBaseline, 'omitnan'));
fprintf('F1 Score: %f\n', mean(F1Baseline, 'omitnan'));

fprintf('\nImproved Model Metrics:\n');
fprintf('Precision: %f\n', mean(precisionImproved, 'omitnan'));
fprintf('Recall: %f\n', mean(recallImproved, 'omitnan'));
fprintf('F1 Score: %f\n', mean(F1Improved, 'omitnan'));