clear all; close all; clc
addpath('./util')

% Test variables
pause_length = 0.5; 
pulse_length = 0;
noise_samples = 15;
model_type = 'DNN';
trajectory = 'circle';
record = false;
demo = false;


pause_idx = 1;
trajectory_name = ['./inference/',trajectory,'_trajectory.mat'];
inputs_name = ['./inference/',model_type,'_',trajectory,'_trajectory_inputs.mat'];
pause_name = ['./inference/',trajectory,'_pause.mat'];
motor_name = ['./inference/',trajectory,'_motor.mat'];
save_points_name = ['./inference/',trajectory,'_p_'];

%% Initial setup

% Save path
currentDateTime = datetime('now');
dirName = datestr(currentDateTime, '_yyyy_mm_dd_HH_MM_SS');
save_path = ['./experiments/',trajectory, dirName];
photo_path = [save_path,'/pictures'];

if record
    mkdir(save_path);
    mkdir(photo_path);
end

% Camera setup
cam = videoinput('winvideo', 1);
set(cam, 'FramesPerTrigger', Inf);
set(cam, 'ReturnedColorspace', 'rgb')
cam.FrameGrabInterval = 1;  % Grab one frame every second

% Hardware initialization
arm = robotArm();
motor = armMotor();

% Loading trajectory infox
l_delta = load(inputs_name).output;
num_points=size(l_delta,1);
comp = load('./state/comp.mat').comp;
if demo
    pause_mat = load(pause_name).pause_mat;
    motor_mat = load(motor_name).motor_mat;
end

if record
    copyfile(trajectory_name, save_path);
    copyfile(inputs_name, save_path);
end
%% Loop and collect data

% Initialize output
output = cell(num_points + 1, 9);
output(1,:) = {'date and time','p',...
    'x_end_avg','y_end_avg','z_end_avg','qx_end_avg','qy_end_avg','qz_end_avg','qw_end_avg'};

fprintf('Starting test\n');
   
% arm.reset_arm();

for p = 1:num_points
   
    fprintf('Waypoint: %d/%d \n', p, num_points);
    fprintf('========================\n');
    
    if demo && pause_mat(p) == -1
        input('Press enter to keep running')
    end

    % Set arm to new pose
    arm.set_pos(double(comp+l_delta(p,:)))
    
    % Check for pause at waypoint
    if demo && (pause_mat(p) ~= -1 && pause_mat(p) ~= 0)
        pause(pause_mat(p))
    end

    % Check for motor pulse
    if demo && (motor_mat(p) > 0)
        pause(0.25)
        disp(motor_mat(p))
        motor.pulse(motor_mat(p))
        pause(0.25)
    end

    if record

        % Pause for equillibirum
        pause(pause_length)
    
        % Sample to reduce noise
        S = zeros(noise_samples,7);
        
        for i = 1:noise_samples
            tool = arm.get_pose();
            S(i,:) = [tool.x, tool.y, tool.z, tool.qx, tool.qy, tool.qz, tool.qw];
            pause(1/60);
        end
        
        % Write to output
        output(p+1,1:2) = {datetime,p};
        output(p+1,3:9) = num2cell(mean(S,1));
        
        if record
            % Take a photo
            img = getsnapshot(cam);
            % Save the image to disk.
            filename = sprintf('/waypoint_%d.jpg', p);
            im_path = [photo_path,filename];
            imwrite(img, im_path);
        end
    end

end

% Clean up.
delete(cam)
clear cam

arm.reset_arm();

% Save output file
if record
    writecell(output,[save_path,'/positions.csv'])
    copyfile('./state', save_path);
    copyfile('./trajectory', save_path);
end