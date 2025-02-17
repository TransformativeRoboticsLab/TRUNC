clear all; close all; clc
addpath('./util')

% Test variables
num_points = 100;
repeat = 5;
pause_length = 3;
random_order = false;
pulse_length = 0;
noise_samples = 15;
movment_time = 2;
anti_slack = true;

%% Initial setup

% Save path
currentDateTime = datetime('now');
dirName = datestr(currentDateTime, 'yyyy_mm_dd_HH_MM_SS');
save_path = ['./repeatability/', dirName];
photo_path = [save_path,'/pictures'];
mkdir(save_path);
mkdir(photo_path);

% Camera setup
cam = videoinput('winvideo', 1);
set(cam, 'FramesPerTrigger', Inf);
set(cam, 'ReturnedColorspace', 'rgb')
cam.FrameGrabInterval = 1;  % Grab one frame every second

% Hardware
arm = robotArm();
motor = armMotor();
l_delta = load("./trajectory/delta_fast_repeat.mat").delta_fast;
comp = load("./state/comp.mat").comp;

%% Loop and collect data

% Initialize output
output = cell(num_points*repeat + 1, 11);
output(1,:) = {'date and time','Repeat num','Test num','p_idx',...
    'x_end_avg','y_end_avg','z_end_avg','qx_end_avg','qy_end_avg','qz_end_avg','qw_end_avg'};

fprintf('Starting test\n');
fprintf('Total %d samples\n', repeat);
fprintf('Estimated test duration: %0.3f hours\n',  num_points * (pause_length + movment_time) * repeat/ 3600);

% Run test loop
for r = 1:repeat
    fprintf('Repetition %d/%d\n', r, repeat);
   
    % Determine the order of points
    if random_order
        % Visit points in a random order
        pointOrder = randperm(num_points);
    else
        % Visit points in a fixed order
        pointOrder = 1:num_points;
    end

    % arm.reset_arm();
    
    slack = zeros([1,9]);

    for p_idx = 1:num_points
        
        slack = 0.6.*slack;

        p = pointOrder(p_idx); % Get the actual point index

        fprintf('Point: %d/%d Point id: %d \n', p_idx, num_points, p);
        fprintf('========================\n');
        
        % Set arm to new pose
        pos = comp+l_delta(p,:);
        arm.set_pos(comp+l_delta(p,:)+slack)
        
        % Pause for equillibirum
        pause(pause_length)

        % Anti-slack compensation
        slack_idx = [3,2,1,6,5,4,9,8,7];
        p_t = 3.5;
        r_t = 1;

        if anti_slack
            % Loop through each cable
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

        % Sample to reduce noise
        S = zeros(noise_samples,7);
                                                                                                                                                                                                                                                                                                       
        for i = 1:noise_samples
            tool = arm.get_pose();
            S(i,:) = [tool.x, tool.y, tool.z, tool.qx, tool.qy, tool.qz, tool.qw];
            pause(1/60);
        end
        
        % Write to output
        output((r-1)*num_points+p+1,1:4) = {datetime,r,p,p_idx};
        output((r-1)*num_points+p+1,5:11) = num2cell(mean(S,1));

        % Take a photo
        img = getsnapshot(cam);
        % Save the image to disk.
        filename = sprintf('/pose_%d_repetition_%d.jpg', p, r);
        im_path = [photo_path,filename];
        imwrite(img, im_path);

    end
end

% Reset
arm.reset_arm();

% Clean up.
delete(cam)
clear cam

% Save output file
writecell(output,[save_path,'/positions.csv'])
copyfile('./state', save_path);
copyfile('./trajectory', save_path);

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