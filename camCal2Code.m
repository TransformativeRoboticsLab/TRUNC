% Auto-generated by cameraCalibrator app on 02-Aug-2024
%-------------------------------------------------------


% Define images to process
imageFileNames = {'C:\Users\hajjafar.k\aux-net\matlab\training\camCal2v2\CAM2\WIN_20240802_13_53_55_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal2v2\CAM2\WIN_20240802_13_53_58_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal2v2\CAM2\WIN_20240802_13_54_00_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal2v2\CAM2\WIN_20240802_13_54_03_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal2v2\CAM2\WIN_20240802_13_54_05_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal2v2\CAM2\WIN_20240802_13_54_10_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal2v2\CAM2\WIN_20240802_13_54_13_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal2v2\CAM2\WIN_20240802_13_54_15_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal2v2\CAM2\WIN_20240802_13_54_19_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal2v2\CAM2\WIN_20240802_13_54_22_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal2v2\CAM2\WIN_20240802_13_54_24_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal2v2\CAM2\WIN_20240802_13_54_28_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal2v2\CAM2\WIN_20240802_13_54_33_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal2v2\CAM2\WIN_20240802_13_54_35_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal2v2\CAM2\WIN_20240802_13_54_37_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal2v2\CAM2\WIN_20240802_13_54_40_Pro.jpg',...
    };
% Detect calibration pattern in images
detector = vision.calibration.monocular.CheckerboardDetector();
[imagePoints, imagesUsed] = detectPatternPoints(detector, imageFileNames, 'HighDistortion', true);
imageFileNames = imageFileNames(imagesUsed);

% Read the first image to obtain image size
originalImage = imread(imageFileNames{1});
[mrows, ncols, ~] = size(originalImage);

% Generate world coordinates for the planar pattern keypoints
squareSize = 23;  % in units of 'millimeters'
worldPoints = generateWorldPoints(detector, 'SquareSize', squareSize);

% Calibrate the camera
[cameraParams2, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'millimeters', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
    'ImageSize', [mrows, ncols]);

%Save the cameraParams into a separate file:
save('params.mat', "cameraParams2", "-append")

% % View reprojection errors
% h1=figure; showReprojectionErrors(cameraParams);
% 
% % Visualize pattern locations
% h2=figure; showExtrinsics(cameraParams, 'CameraCentric');
% 
% % Display parameter estimation errors
% displayErrors(estimationErrors, cameraParams);
% 
% % For example, you can use the calibration data to remove effects of lens distortion.
% undistortedImage = undistortImage(originalImage, cameraParams);
% 
% % See additional examples of how to use the calibration data.  At the prompt type:
% % showdemo('MeasuringPlanarObjectsExample')
% % showdemo('StructureFromMotionExample')
