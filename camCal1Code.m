% Auto-generated by cameraCalibrator app on 02-Aug-2024
%-------------------------------------------------------


% Define images to process
imageFileNames = {'C:\Users\hajjafar.k\aux-net\matlab\training\camCal1v2\CAM1\WIN_20240802_13_55_28_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal1v2\CAM1\WIN_20240802_13_55_32_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal1v2\CAM1\WIN_20240802_13_55_35_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal1v2\CAM1\WIN_20240802_13_55_37_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal1v2\CAM1\WIN_20240802_13_55_39_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal1v2\CAM1\WIN_20240802_13_55_42_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal1v2\CAM1\WIN_20240802_13_55_44_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal1v2\CAM1\WIN_20240802_13_55_46_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal1v2\CAM1\WIN_20240802_13_55_50_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal1v2\CAM1\WIN_20240802_13_55_53_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal1v2\CAM1\WIN_20240802_13_55_57_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal1v2\CAM1\WIN_20240802_13_55_59_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal1v2\CAM1\WIN_20240802_13_56_09_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal1v2\CAM1\WIN_20240802_13_56_15_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal1v2\CAM1\WIN_20240802_13_56_18_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal1v2\CAM1\WIN_20240802_13_56_20_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal1v2\CAM1\WIN_20240802_13_56_23_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal1v2\CAM1\WIN_20240802_13_56_26_Pro.jpg',...
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
[cameraParams1, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'millimeters', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
    'ImageSize', [mrows, ncols]);

%We want to overwrite the same file so that we can start fresh when running
%the calib() function again in the april tag python translation file.
%Save the cameraParams into a separate file:
save('params.mat', "cameraParams1")

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