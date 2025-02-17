clear all; close all; clc
addpath('./util')

% Test variables
pause_length = 3;
pulse_length = 0;
noise_samples = 15;
movment_time = 2;
traj_start = 124;
anti_slack = true;
slack_spacing = 2;

%% Initial setup

% Save path
currentDateTime = datetime('now');
dirName = datestr(currentDateTime, 'yyyy_mm_dd_HH_MM_SS');
save_path = ['./data/', dirName];
photo_path = [save_path,'/pictures'];
mkdir(save_path);
mkdir(photo_path);

% Save relevant info files
copyfile('./state', save_path);
copyfile('./trajectory', save_path);

% Camera setup
cam = videoinput('winvideo', 1);
set(cam, 'FramesPerTrigger', Inf);
set(cam, 'ReturnedColorspace', 'rgb')
cam.FrameGrabInterval = 1;  % Grab one frame every second
w
% Hardware
arm = robotArm();
motor = armMotor();
comp = load("./state/comp.mat").comp;

%% Loop and collect data

training_waypoints = load('./trajectory/training_waypoints.mat').points;

num_points = size(training_waypoints,1);
num_trajectories = size(training_waypoints,3);

% Initialize output
output = cell(num_points*num_trajectories + 1, 19);
output(1,:) = {'date and time','Trajectory','Waypoint',...
    'x_end_avg','y_end_avg','z_end_avg','qx_end_avg','qy_end_avg','qz_end_avg','qw_end_avg',...
    'l0','l1','l2','l3','l4','l5','l6','l7','l8'};

fprintf('Starting test\n');
fprintf('Estimated test duration: %0.3f hours\n',  num_points * (pause_length + movment_time) * num_trajectories/ 3600);

% Save header
writecell(output(1,:),[save_path,'/positions.csv'])

% Run test loop
for r = traj_start:num_trajectories
    fprintf('Trajectory %d/%d\n', r, num_trajectories);

    arm.reset_arm();

    slack = zeros([1,9]);

    for p = 1:num_points

        fprintf('Point: %d/%d\n', p,num_points);
        fprintf('========================\n');
        
        delta_pos = training_waypoints(p,:,r); 
        pos = comp+training_waypoints(p,:,r);

        % Anti-slack compensation
        slack_idx = [3,2,1,6,5,4,9,8,7];
        p_t = 3.5;
        r_t = 1;

        if anti_slack && mod(p-1,slack_spacing) == 0
            % Loop through each cable
            disp('Removing slack')
            slack = 0.5.*slack; 
            arm.set_pos(pos+slack)
            pause(pause_length)
            for i = 1:length(slack_idx)
                slack_removed = false;
                tool_0 = arm.get_pose();
                pause(0.5)
                while ~slack_removed
                    idx = slack_idx(i);
                    slack(idx) = slack(idx)-.075;
                    arm.set_pos(pos+slack)
                    tool_i = arm.get_pose();
                    slack_removed = check_movment(tool_0,tool_i,p_t,r_t);
                end
            end
        end

        % Set arm to new pose
        arm.set_pos(pos+slack)
        
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
        output((r-1)*num_points+p+1,1:3) = {datetime,r,p};
        output((r-1)*num_points+p+1,4:10) = num2cell(mean(S,1));
        output((r-1)*num_points+p+1,11:19) = num2cell(delta_pos+slack);
        writecell(output((r-1)*num_points+p+1,:),[save_path,'/positions.csv'],'WriteMode','append')

        % Take a photo
        img = getsnapshot(cam);
        % Save the image to disk.
        filename = sprintf('/trajectory%d_pose_%d.jpg', r, p);
        im_path = [photo_path,filename];
        imwrite(img, im_path);

    end
end

% Reset arm and camera
arm.reset_arm();

delete(cam)
clear cam

%% Helper function

function d = quaternionDistance(q1, q2)
    % Ensure the quaternions are normalized
    q1 = q1 / norm(q1);
    q2 = q2 / norm(q2);

    % Calculate the dot product (cosine of the angle)
    dotProd = abs(dot(q1, q2));

    % Calculate the distance
    d = 1 - dotProd;
end


function slack_removed = check_movment(t_0,t_i,p_t,r_t)
slack_removed = false;

p_0 = 1000.*[t_0.x,t_0.y,t_0.z];
p_i = 1000.*[t_i.x,t_i.y,t_i.z];

r_0 = [t_0.qw,t_0.qx,t_0.qy,t_0.qz];
r_i = [t_i.qw,t_i.qx,t_i.qy,t_i.qz];

d = sqrt(sum((p_0-p_i).^2));
d_r = rad2deg(quaternionDistance(r_0,r_i));

if d >= p_t || d_r >= r_t
    slack_removed = true;
end

end