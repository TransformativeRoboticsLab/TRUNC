close all; clc; clear all;
% 
% % Camera setup
% cam1 = videoinput('winvideo', 1);
% set(cam1, 'FramesPerTrigger', Inf);
% set(cam1, 'ReturnedColorspace', 'rgb')
% cam1.FrameGrabInterval = 1;  % Grab one frame every second
% 
% img1 = getsnapshot(cam1);
% 
% imshow(img1)
% 
% % Camera setup
% cam2 = videoinput('winvideo', 2);
% set(cam2, 'FramesPerTrigger', Inf);
% set(cam2, 'ReturnedColorspace', 'rgb')
% cam2.FrameGrabInterval = 1;  % Grab one frame every second
% 
% img2 = getsnapshot(cam2);
% 
% imshow(img2)
% 
% % Camera setup
% cam0 = videoinput('winvideo', 3);
% set(cam0, 'FramesPerTrigger', Inf);
% set(cam0, 'ReturnedColorspace', 'rgb')
% cam0.FrameGrabInterval = 1;  % Grab one frame every second
% 
% img0 = getsnapshot(cam0);
% 
% imshow(img0)

% Call the main function to run the script
main();

%Function used to calibrate the cameras and to retrieve their intrinsics.
function calib()
    disp('running camera calibrations')
    run('camCal1Code.m')
    run('camCal2Code.m')
    run('camCal3Code.m')

    disp('ending calibrations')
end

%Turn on and initialize the mocap system.
function arm = mocap()
    addpath('./util/');
    arm = robotArm();
end

% Initialize the cameras
function cameras = on()
    % Define camera indices
    camera_indices = [1, 2, 3];  % Update with your specific camera indices
    numCams = length(camera_indices);
    cameras = cell(1,numCams);
    for i = 1:numCams
        cam = videoinput('winvideo', camera_indices(i), 'MJPG_3840x2160');
        set(cam, 'FramesPerTrigger', 1);
        set(cam, 'TriggerRepeat', Inf);
        set(cam, 'ReturnedColorSpace', 'rgb');
        set(cam, 'Timeout', 10); % 10 second timeout for the cameras
        triggerconfig(cam, 'manual');
        start(cam);

        % Check if camera is started successfully
        if ~isrunning(cam)
            error(['Error: Could not start camera ' ...
                num2str(camera_indices(i))]);
        else
            disp(['Camera ' num2str(camera_indices(i)) ' is ready.']);
            cameras{i} = cam;
        end

        % Delay to help improve camera initialization
        pause(0.1);
    end
end

% Turn off the cameras
function off(cameras)
    % Stop and delete all cameras
    for i = 1:length(cameras)
        stop(cameras{i});
        delete(cameras{i});
    end

    disp('Cameras turned off.');
end

% Capture images from each camera and retrieve images from Mocap
function images = capture(cameras)%, arm)
    images = cell(1, length(cameras));
    %tool = arm.get_pose;
    for i = 1:length(cameras)
        trigger(cameras{i});
        disp("Triggered")
        images{i} = getdata(cameras{i});
    end
end

%Function to create a matrix relative to tag to world.
function tag2world = tag2world_conv(cam2world, tagR, tagP)
    tagPtrans = tagP';

    tag2cam = eye(4, 4);

    %add the rotation to the matrix
    tag2cam(1:3,1:3) = tagR;
    
    %add the position to the matrix
    tag2cam(1:3,4) = tagPtrans;

    tag2world = cam2world * tag2cam;
end

% Process images to find AprilTags and their positions/orientations
function [finalP, finalQ] = getPosOr(images)

    finalP = {};
    finalQ = {};
    %paramters for each camera intrinsics
    load('params.mat','cameraParams3', 'cameraParams1', 'cameraParams2');
    intrinsics = {cameraParams1.Intrinsics, cameraParams2.Intrinsics, ...
        cameraParams3.Intrinsics};
    %Matrices for camera to world for each camera
    load('H_camera2world.mat','HInv');

    for i = 1:length(images)
        % Convert image to grayscale
        image = rgb2gray(images{i});

        %Store current cam2world matrix
        cam2world = HInv{i};

        %modify contrast
        %image = image -100;
        %image = imadjust(image,[0.2 0.4],[]);

        % Blur the image for better edge detection
        img_blur = imgaussfilt(image, 0.75);

        %% % Apply adaptive thresholding
        % img_blur = imbinarize(img_blur, 'adaptive',... 
        %      'ForegroundPolarity', 'dark', 'Sensitivity', 0.7);
        img_blur = imbinarize(img_blur, 'adaptive',... 
             'ForegroundPolarity', 'bright', 'Sensitivity', 0.75);

        % Display the image (optional)
        figure;
        imshow(img_blur);
        title('Processed Image');
        pause(1);  % Pause to display the image

        %tagSize = 137;
        %tagSize = 75;
        tagSize = 29;

        % Detect AprilTags
        %[ids, corners, centers, Hs] = readAprilTag(img_blur,'Tag16h5');
        [id, loc, pose, fam] = readAprilTag(img_blur,'Tag16h5',intrinsics{i},tagSize);

        % Process each detected tag
        if isempty(id)
            disp(['No IDs were detected in camera ' num2str(i) '.']);
            continue;
        end
        for j = 1:length(id)
             
            tag_id = id(j);
            disp(['Detected ID in camera ' num2str(i) ': ' num2str(tag_id)]);
            if tag_id ~= 0 && tag_id ~= 1 && tag_id ~= 2 %&& tag_id ~= 4
                continue;
            end
            
            %save rotation matrix
            rotation_matrix = pose(j).R;
            
            pose(j).Translation

            %find the tag to world matrix
            tag2world = tag2world_conv(cam2world, rotation_matrix, pose(j).Translation);

            %Save the 3x3 rotation matrix and the 1x3 position (with
            %transposing it)
            t2w_R = tag2world(1:3,1:3);
            t2w_P = tag2world(1:3,4)';

            % Convert rotation matrix to quaternion
            quat_Rotation = rotm2quat(t2w_R);

            disp(['Tag ID ' num2str(tag_id) ': Tag to World Position' ...
                '(X, Y, Z) = (' num2str(t2w_P) ')']);

            disp(['Quaternion Orientation of Tag to World '... 
                num2str(tag_id) ': ' num2str(quat_Rotation)]);

            % Store final positions and orientations
            finalP{i} = t2w_P;
            finalQ{i} = quat_Rotation;

        end
    end
end

function cameraReset()
    % Find all existing camera objects
    vid_objs = imaqfind;
    
    % Check if any camera objects are found
    if ~isempty(vid_objs)
        % Loop through each camera object and stop/delete it
        for i = 1:length(vid_objs)
            stop(vid_objs(i));
            delete(vid_objs(i));
        end
        disp('Existing camera objects turned off.');
    else
        disp('No existing camera objects found.');
    end
end

% Main function to run the script
function main()
    
    %% Uncomment calib() to calibrate the camera intrinsics
    %calib()

    % Stop and delete all cameras before starting
    cameraReset();

    % Initialize cameras
    cameras = on();

    %Initialize the mocap
    %arm = mocap();

    %timer start
    tic;

    % Capture images
    %[images,tool] = capture(cameras,arm);
    images = capture(cameras);

    % Process images to find AprilTags and their positions/orientations
    [processedP, processedQ] = getPosOr(images);

    %end timer
    proTime = toc;
    fprintf('Time taken to process images: %.3f milliseconds\n', proTime * 1000);

    disp(processedP)
    % fprintf('Mocap location (x,y,z): (%0.3f, %0.3f, %0.3f)\n', ...
    %      tool.z * 1000, tool.x * 1000, tool.y * 1000);

    %Select which camera you want to compare the mocap cam to
    tagLocation = processedP{1};

    % fprintf('Delta (Mocap - April): (%0.1f, %0.1f, %0.1f)\n', ...
    %      tool.z * 1000 - tagLocation(1), tool.x * 1000 - tagLocation(2), ...
    %      tool.y * 1000 - tagLocation(3));

    %Average out the cameras' values to see what we get:
    avgPMat = cell2mat(processedP');
    avgP = mean(avgPMat);
    avgQMat = cell2mat(processedQ');
    avgQ = mean(avgQMat);

    disp(avgP)
    %disp(processedQ)

    % fprintf('Delta (Mocap - AvgApril): (%0.1f, %0.1f, %0.1f)\n', ...
    %  tool.z * 1000 - avP(1), tool.x * 1000 - avP(2), ...
    %  tool.y * 1000 - avP(3));

    filename = 'aprilTagAverages.mat';

    if (length(processedP) == 3)
        % Used to save the average Position and Quaternion to a .mat file.
        % Check if the file already exists
        if isfile(filename)
            % Load existing variables from the file
            existingData = load(filename);
            % Combine new data with existing data (example: appending to cell arrays)
            avgP = [existingData.avgP; avgP];
            avgQ = [existingData.avgQ; avgQ];
        end
        save(filename, "avgP", "avgQ")
    end
    

    % Turn off cameras
    off(cameras);
end


