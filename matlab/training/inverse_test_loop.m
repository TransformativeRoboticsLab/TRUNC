clear all; close all; clc

% This section reads the data 
addpath(genpath('C:\Users\trlab\Desktop\Optitrack\auxarm_optitrack_motor_matlab\data recording scripts\optitrack motor control'))
addpath('C:\Users\trlab\Desktop\Optitrack\auxarm_optitrack_motor_matlab\data recording scripts\NatNet_SDK_3.1\NatNet_SDK_3.1\NatNetSDK\Samples\Matlab')

motor_testing = 1; %run motors; 0 to test code and 1

num_points = 7500;
repeat = 1;
pause_length = 1.5;

home_position = load("home_position.mat").home_position;
compress = load("compress.mat").compress;

md = readtable("predicted_motor_inputs_fast_7500.csv");
motor_pos = load("motor_pos_inverse_fast.mat").motor_pos_fast;


% Connect to servos and initialize the arm
if ~exist('port','var')
    port = serialport("COM6", 9600);
    channels = 0:11;
    pos = zeros(12,1);
    global position;
    if motor_testing
        initialize_servos(port, channels, pause_length,pos,home_position);
        pause(5);
        stop_servos(port, channels);
    end
end

% Connect to NatNet
pause(1);

if ~exist('nnc','var')
    nnc = connect_to_natnet();
end

% Compress
if motor_testing
    start_pos = home_position;
    for scale = linspace(0.1,1,10)
        for motor = 0:11
            set_servo_position_auxarm(port,motor,start_pos(motor+1)+scale*(compress(motor+1)-start_pos(motor+1)),position);
        end
        pause(0.15)
    end
end

pause(8);

ee = nnc.getFrame().RigidBody(1); %End Effector

xe = {ee.x, ee.y, ee.z}
qe = {ee.qx, ee.qy, ee.qz, ee.qw};

%% Loop and collect data

% Initialize output
init_frame = nnc.getFrame();
output = cell(num_points*repeat + 1, 15); % + 3 + 4 + marker_count * 3);
output(1,:) = {'date and time','Repeat num','Test num','dtm_13','dtm_24','de_13','de_24','dl',...
    'x_end_avg','y_end_avg','z_end_avg','qx_end_avg','qy_end_avg','qz_end_avg','qw_end_avg'};

fprintf('Starting test\n');
fprintf('Total %d samples\n', repeat);
fprintf('Estimated test duration: %0.3f hours\n',  num_points * (pause_length + 1.5 + .5) * repeat/ 3600); % factor 2 comes from initialize_servos also pausing

ee = nnc.getFrame().RigidBody(1); %End Effector

for r = 1:repeat
    fprintf('Repetition %d/%d\n', r, repeat);
    for p = 1:num_points
        % Verify that Optitrack is connected
        if (~nnc.IsConnected)
            error('Not connected :(');
            break;
        end

        fprintf('Point %d/%d\n', p, num_points);
        fprintf('========================\n');
        
        % Command each servo based on motor position for that iteration
        if motor_testing
            start_pos = position.';
            t_pause = 0.1;
            v = 20;
            d = motor_pos(p,:)-start_pos;
            d_abs = sqrt(sum(d.^2));
            n = round(d_abs/(v*t_pause));
            for scale = linspace(0.1,1,n)
                for motor = 0:11
                    set_servo_position_auxarm(port,motor,start_pos(motor+1)+scale*d(motor+1),position);
                end
                pause(t_pause)
            end
        end
        
        pause(pause_length)

        % Sample 15 times to reduce noise
        S = zeros(15,7);
        
        for i = 1:15
            % frame = nnc.getFrame();
            ee = nnc.getFrame().RigidBody(1); %End Effector
            mm = nnc.getFrame().RigidBody(2); %Mid Effector
            tt = nnc.getFrame().RigidBody(3); %Top Effector
            S(i,:) = [ee.x, ee.y, ee.z, ee.qx, ee.qy, ee.qz, ee.qw];
            pause(1/60);
        end
        
        % Write to output
        output((r-1)*num_points+p+1,1:3) = {datetime,r,p};
        output((r-1)*num_points+p+1,4:8) = table2cell(md(p,:));
        output((r-1)*num_points+p+1,9:15) = num2cell(mean(S,1));

    end
end

% Decrease again servo speed for path
port = serialport("COM6", 9600);
channels = 0:11;

initialize_servos(port, channels, pause_length,pos,home_position);
pause(5)
stop_servos(port, channels);

% Save output file
save_path = append("./inverse_validate_7500_",datestr(datetime('now'),'mm_dd_yyyy'),".csv");
writecell(output,save_path)