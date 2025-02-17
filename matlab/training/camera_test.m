cam = videoinput('winvideo', 1);
set(cam, 'FramesPerTrigger', Inf);
set(cam, 'ReturnedColorspace', 'rgb')
cam.FrameGrabInterval = 1;  % Grab one frame every second
% Start the camera.
start(cam)
% Define the number of frames you want to capture.
numFrames = 10;  % For example, capture 10 frames
% Capture the frames.
for i = 1:numFrames
   % Acquire a single image.
   img = getsnapshot(cam);
   % Save the image to disk.
   filename = sprintf('./image_%d.jpg', i);
   imwrite(img, filename);
   % Wait for 1 second.
   pause(1);
end
% Stop the camera.
stop(cam)
% Clean up.
delete(cam)
clear cam