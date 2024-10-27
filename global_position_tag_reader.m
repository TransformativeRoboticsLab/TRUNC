%This code is to detect the global position apriltag and retrieve a value
%from that.
close all; clc; clear all;
% Call the main function to run the script
main();

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

% Initialize the cameras
function cameras = on()
    % Define camera indices
    camera_indices = [1, 2, 3];  % Update with your specific camera indices

    cameras = {};
    for i = 1:length(camera_indices)
        cam = videoinput('winvideo', camera_indices(i), 'MJPG_3840x2160');
        cam.FrameGrabInterval = 2;
        set(cam, 'FramesPerTrigger', 1);
        set(cam, 'TriggerRepeat', Inf);
        set(cam, 'ReturnedColorSpace', 'rgb');
        set(cam, 'Timeout', 30);
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
    end
end

% Capture images from each camera
function images = capturePic(cameras)
    images = cell(1, length(cameras));
    for i = 1:length(cameras)
             trigger(cameras{i});
        images{i} = getdata(cameras{i});
    end
end

% Process images to find AprilTags and their positions/orientations
function [finalP, finalQ] = getPosOr(images)

    finalP = {};
    finalQ = {};
    load('params.mat','cameraParams3', 'cameraParams1', 'cameraParams2');
    intrinsics = {cameraParams1.Intrinsics, cameraParams2.Intrinsics, ...
        cameraParams3.Intrinsics};

    %intrinsics{2} = cameraIntrinsics([2.302031922312557e+03,2.299706189304423e+03],intrinsics{2}.PrincipalPoint, intrinsics{2}.ImageSize);

    for i = 1:length(images)
        % Convert image to grayscale
        image = rgb2gray(images{i});

        %modify contrast
        image = image -100;
        image = imadjust(image,[0.2 0.4],[]);

        % Blur the image for better edge detection
        img_blur = imgaussfilt(image, 0.75);

        % % Apply adaptive thresholding
        img_blur = imbinarize(img_blur, 'adaptive',... 
             'ForegroundPolarity', 'dark', 'Sensitivity', 0.7);

        % % Display the image (optional)
        % figure;
        % imshow(img_blur);
        % title('Processed Image');
        % pause(1);  % Pause to display the image

        %tagSize = 137;
        tagSize = 75;

        % Detect AprilTags
        %[ids, corners, centers, Hs] = readAprilTag(img_blur,'Tag16h5');
        [id, loc, pose, fam] = readAprilTag(img_blur,'Tag16h5',intrinsics{i},tagSize);

        [id2,loc2] = readAprilTag(img_blur,'Tag16h5');
        disp(id);
        disp(id2);


        % Process each detected tag
        if isempty(id)
            disp(['No IDs were detected in camera ' num2str(i) '.']);
            continue;
        end
        for j = 1:length(id)
             
            tag_id = id(j);
            disp(['Detected ID in camera ' num2str(i) ': ' num2str(tag_id)]);
            if tag_id ~= 4
                continue;
            end

            disp(['Tag ID ' num2str(tag_id) ': Position (X, Y, Z) = (' ...
                num2str(pose(j).Translation) ')']);

            %save rotation matrix
            rotation_matrix = pose(j).R;

            disp(['Rotation Matrix ' num2str(tag_id) ': ']);
            disp(rotation_matrix);

            % Store final positions and orientations
            finalP{i} = pose(j).Translation;
            finalQ{i} = rotation_matrix;

        end
    end
end

% Turn off the cameras
function off(cameras)
    % Stop and delete all cameras
    for i = 1:length(cameras)
        stop(cameras{i});
        delete(cameras{i});
        clear cameras{i}
    end

    disp('Cameras turned off.');
end

% Capture images from each camera
function images = capture(cameras)
    images = cell(1, length(cameras));
    for i = 1:length(cameras)
        trigger(cameras{i});
        images{i} = getdata(cameras{i});
    end
end

%   This function will create a [[R,P][0,1]] Matrix and inverse it to be saved.
function HInv = inverseH(P,R)
    for i = 1:length(R)
        %set the matrix to be 4x4
        matrix = eye(4, 4);

        R_matrix_i = R{i};
        P_matrix_i = P{i}'; %transpose it
        
        R_inv = R_matrix_i';
        P_inv = -R_inv*P_matrix_i;

        %add the rotation to the matrix
        matrix(1:3,1:3) = R_inv;
    
        %add the position to the matrix
        matrix(1:3,4) = P_inv;

        %Create the inverse of the matrix
        HInv{i} = matrix;
    
    end
end

% Main function to run the script
function main()
    disp("Running global position april tag reader script...")

    % Stop and delete all cameras before starting
    cameraReset();

    % Initialize cameras
    cameras = on();


    % Capture images
    images = capturePic(cameras);

    % Process images to find AprilTags and their positions/orientations
    [processedP, processedQ] = getPosOr(images);

    HInv = inverseH(processedP,processedQ);

    %Save the cameraParams into a separate file:
    save('H_camera2world.mat', "HInv")

    % Turn off cameras
    off(cameras);

    disp("Cameras are turned off.")
end