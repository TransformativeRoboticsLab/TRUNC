close all; clc; clear all;

home_pos = load('./state/home_measured.mat').pos;
point_density = load('./state/point_density.mat').dist_per_point;

%% First trajectory (circle)

export_traj = false;

tool_rot = eye(3);
tool_quat = rotm2quat(tool_rot);

n = 20;
r = 90;
z_plane = 80;
theta = linspace(0,2*pi,n).';

x_c = r.*cos(theta);
y_c = r.*sin(theta);
z_c = repmat(z_plane,size(theta));

wp = [zeros(1,3), tool_quat;
      x_c, y_c, z_c, repmat(tool_quat,[n,1])];
wp = wp + home_pos;
wp = interp_waypoints(wp,100,"cubic");

figure(1); clf; hold on; grid on

plot3(wp(:,1), wp(:,2), wp(:,3), 'x-', 'LineWidth', 1.5); % 'k:' makes the line black and dotted, 'LineWidth' sets the thickness

xlabel('X-axis'); % Label for the x-axis
ylabel('Y-axis'); % Label for the y-axis
zlabel('Z-axis'); % Label for the z-axis

if export_traj
    save('./inference/circle_trajectory.mat','wp')
end

%% Second trajectory (triangle)

export_traj = true;

tool_rot = eye(3);
tool_quat = rotm2quat(tool_rot);

l = 80;
d = 20;
z_plane = 90;

wp_xyz = [0,0,0; % Origin
          0,l,z_plane; % #1 (start)
          0,l,z_plane-d;
          0,l,z_plane; % #1 (end)
          -l*sin(pi/3),-l*cos(pi/3),z_plane; % #2 (start)
          -l*sin(pi/3),-l*cos(pi/3),z_plane-d;
          -l*sin(pi/3),-l*cos(pi/3),z_plane; % #2 (start)
          l*sin(pi/3),-l*cos(pi/3),z_plane; % #3 (start)
          l*sin(pi/3),-l*cos(pi/3),z_plane-d;
          l*sin(pi/3),-l*cos(pi/3),z_plane; % #3 (end)
          0,l,z_plane; % #1 (start)
          ];

n = length(wp_xyz);

wp = [wp_xyz,repmat(tool_quat,[n,1])];
wp = interp_waypoints(wp,100,"linear");

theta = deg2rad(60); % Example angle in degrees converted to radians
Rz = [cos(theta) -sin(theta) 0;
      sin(theta) cos(theta) 0;
      0 0 1];

wp_rot = Rz*wp(:,1:3).';
wp(:,1:3) = wp_rot.';
wp(:,1:3) = wp(:,1:3) + home_pos(1:3);

figure(2); clf; hold on; grid on; axis equal

plot3(wp(:,1), wp(:,2), wp(:,3), 'x-', 'LineWidth', 1.5); % 'k:' makes the line black and dotted, 'LineWidth' sets the thickness

xlabel('X-axis'); % Label for the x-axis
ylabel('Y-axis'); % Label for the y-axis
zlabel('Z-axis'); % Label for the z-axis

title('3D Line Plot with Grid'); % Title for the plot

if export_traj
    save('./inference/triangle_trajectory.mat','wp')
end

view([-15,30])

%% Third trajectory (steps)

export_traj = true;

tool_0_rot = eye(3);
tool_0_quat = rotm2quat(tool_0_rot);


dy = -160/3;
dz = 25;
z0 = 40;
y0 = 80;

% First set of waypoints
wp_xyz = [0,0,0; % Origin
          0,y0,z0;
          0,y0+dy,z0;
          0,y0+dy,z0+dz;
          0,y0+2*dy,z0+dz;
          0,y0+2*dy,z0+2*dz;
          0,y0+3*dy,z0+2*dz;]; % #1 (start)

n = length(wp_xyz);

wp = [wp_xyz,repmat(tool_0_quat,[n,1])];
wp(:,1:3) = wp(:,1:3) + home_pos(1:3);

wp = interp_waypoints(wp,100,"linear");

figure(3); clf; hold on; grid on; axis equal;

plot3(wp(:,1), wp(:,2), wp(:,3), 'x-', 'LineWidth', 1.5); % 'k:' makes the line black and dotted, 'LineWidth' sets the thickness

xlabel('X-axis'); % Label for the x-axis
ylabel('Y-axis'); % Label for the y-axis
zlabel('Z-axis'); % Label for the z-axis

title('3D Line Plot with Grid'); % Title for the plot

if export_traj
    save('./inference/line_trajectory.mat','wp')
end

view([0,0])


%% Mother board trajectory

export_traj = true;

tool_rot = eye(3);
tool_quat = rotm2quat(tool_rot);

z_hop = [0,0,0];
z_over = [0,0,1];
z_drill = [0,0,-12];
z_drill_2 = [0,0,-9];
z_off = [0,0,4.25];
z_start_off = [0,0,5];
z_arc = 10;
t_drill = 14*(15/20);
offset = 0;
off_start = [0,-25,0];
n_points = 30;

p2 = [24.02,117.07-10,347.32]-home_pos(1:3)+z_off;
p1 = [-82.32,114.60-10,346.2]-home_pos(1:3)+z_off;
p_mat = [p1;p2];
p_mean = mean(p_mat,1);
p_mat = p_mat + home_pos(1:3);

start = p1 + off_start+z_hop+z_start_off;

% First set of waypoints
wp_1 = [0,0,0;
        start];
n_1 = size(wp_1,1);
wp_1 = [wp_1,repmat(tool_quat,[n_1,1])];
wp_1 = interp_waypoints(wp_1,n_points,"linear");
% Define pause and motor mats
pause_mat_1 = zeros([1,size(wp_1,1)]);
pause_mat_1(end) = -1;
motor_mat_1 = zeros([1,size(wp_1,1)]);

% Second set of waypoints
wp_2 = [start;
        p1 + z_hop + z_over;];
n_2 = size(wp_2,1);
wp_2 = [wp_2,repmat(tool_quat,[n_2,1])];
wp_2 = interp_waypoints(wp_2,n_points,"linear");
% Define pause and motor mats
pause_mat_2 = zeros([1,size(wp_2,1)]);
pause_mat_2(end) = 1;
motor_mat_2 = zeros([1,size(wp_2,1)]);

% Third set of waypoints
wp_3 = [p1 + z_hop;
        p1+z_drill];
n_3 = size(wp_3,1);
wp_3 = [wp_3,repmat(tool_quat,[n_3,1])];
wp_3 = interp_waypoints(wp_3,n_points,"linear");
% Define pause and motor mats
pause_mat_3 = zeros([1,size(wp_3,1)]);
pause_mat_3(end) = 0;
motor_mat_3 = zeros([1,size(wp_3,1)]);
motor_mat_3(end) = t_drill;


% Fourth set of waypoints
wp_4 = [p1;
        p1 + z_hop+z_over;];
n_4 = size(wp_4,1);
wp_4 = [wp_4,repmat(tool_quat,[n_4,1])];
wp_4 = interp_waypoints(wp_4,n_points,"linear");
% Define pause and motor mats
pause_mat_4 = zeros([1,size(wp_4,1)]);
pause_mat_4(end) = 0;
motor_mat_4 = zeros([1,size(wp_4,1)]);
motor_mat_4(end) = 0;


% Fifth set of waypoints
wp_5 = [p1 + z_hop + z_over;
        [p_mean(1),p_mean(2)-offset,p_mean(3)+z_arc];
        p2 + z_hop+ z_over;];
n_5 = length(wp_5);
wp_5 = [wp_5,repmat(tool_quat,[n_5,1])];
wp_5 = interp_waypoints(wp_5,n_points,"cubic");
% Define pause and motor mats
pause_mat_5 = zeros([1,size(wp_5,1)]);
pause_mat_5(end) = 1;
motor_mat_5 = zeros([1,size(wp_5,1)]);
motor_mat_5(end) = 0;        

% Sixth set of waypoints
wp_6 = [p2 + z_hop + z_over;
        p2+ z_hop;
        p2 + z_drill_2;];
n_6 = length(wp_6);
wp_6 = [wp_6,repmat(tool_quat,[n_6,1])];
wp_6 = interp_waypoints(wp_6,n_points,"linear");
% Define pause and motor mats
pause_mat_6 = zeros([1,size(wp_6,1)]);
pause_mat_6(end) = 0;
motor_mat_6 = zeros([1,size(wp_6,1)]);
motor_mat_6(end) = t_drill;

% Seventh set of waypoints
wp_7 = [p2 + z_drill;
        p2 + z_hop + z_over;
        p_mean + [0,-25,0] + z_hop + z_over;];
n_7 = size(wp_7,1);
wp_7 = [wp_7,repmat(tool_quat,[n_7,1])];
wp_7 = interp_waypoints(wp_7,n_points,"linear");
% Define pause and motor mats
pause_mat_7 = zeros([1,size(wp_7,1)]);
pause_mat_7(end) = 0;
motor_mat_7 = zeros([1,size(wp_7,1)]);

% Create full waypoint set and plot
wp = [wp_1;wp_2;wp_3;wp_4;wp_5;wp_6;wp_7];
pause_mat = [pause_mat_1,pause_mat_2,pause_mat_3,pause_mat_4,pause_mat_5,pause_mat_6,pause_mat_7];
motor_mat = [motor_mat_1,motor_mat_2,motor_mat_3,motor_mat_4,motor_mat_5,motor_mat_6,motor_mat_7];

wp(:,1:3) = wp(:,1:3)+home_pos(1:3);

figure(2); clf; hold on; grid on; axis equal

plot3(wp(:,1), wp(:,2), wp(:,3), '-', 'LineWidth', 1.5); % 'k:' makes the line black and dotted, 'LineWidth' sets the thickness
scatter3(p_mat(:,1),p_mat(:,2),p_mat(:,3),'x','red','SizeData',100,'LineWidth',2)

xlabel('X-axis'); % Label for the x-axis
ylabel('Y-axis'); % Label for the y-axis
zlabel('Z-axis'); % Label for the z-axis

title('3D Line Plot with Grid'); % Title for the plot

if export_traj
    save('./inference/motherboard_trajectory.mat','wp')
    save('./inference/motherboard_pause.mat','pause_mat')
    save('./inference/motherboard_motor.mat','motor_mat')
end

view([0,90])

%% Light bulb trajectory

export_traj = true;

tool_rot = eye(3);
tool_quat = rotm2quat(tool_rot);

n_points = 40;

% Measure position of bulb and socket. Some offset exsits
p_bulb = [-23.95,6.60,107.42];
p_socket = [131.85,2.81,209.274];
z_off = [0,0,-25];
z_dip = [0,0,-16];
arc_offset = [0,0,10];
z_retract = [0,0,60];
t_drill = 3.5;
% t_drill = 0;

% Adjust positions based on measured distance
delta_pos = home_pos(1:3)-p_bulb;
p_socket = p_socket + delta_pos + z_off - home_pos(1:3);

% First set of waypoints
wp_1 = [0,0,0;
        p_socket + arc_offset;
        p_socket];
n_1 = size(wp_1,1);
wp_1 = [wp_1,repmat(tool_quat,[n_1,1])];
wp_1 = interp_waypoints(wp_1,n_points,"cubic");
% Define pause and motor mats
pause_mat_1 = zeros([1,size(wp_1,1)]);
pause_mat_1(end) = 1;
motor_mat_1 = zeros([1,size(wp_1,1)]);


% Second set of waypoints
wp_2 = [p_socket;
        p_socket+z_dip];
n_2 = size(wp_2,1);
wp_2 = [wp_2,repmat(tool_quat,[n_2,1])];
wp_2 = interp_waypoints(wp_2,n_points,"cubic");
% Define pause and motor mats
pause_mat_2 = zeros([1,size(wp_2,1)]);
pause_mat_2(end) = 0;
motor_mat_2 = zeros([1,size(wp_2,1)]);
motor_mat_2(end) = t_drill;

% Third set of waypoints
wp_3 = [p_socket+z_dip;
        p_socket+z_retract + [10,0,0];
        p_socket+z_retract + [10,60,0];
        [0,0,0]];
n_3 = size(wp_3,1);
wp_3 = [wp_3,repmat(tool_quat,[n_3,1])];
wp_3 = interp_waypoints(wp_3,n_points,"linear");
% Define pause and motor mats
pause_mat_3 = zeros([1,size(wp_3,1)]);
pause_mat_3(end) = 0;
motor_mat_3 = zeros([1,size(wp_3,1)]);

wp = [wp_1;wp_2;wp_3];
pause_mat = [pause_mat_1,pause_mat_2,pause_mat_3];
motor_mat = [motor_mat_1,motor_mat_2,motor_mat_3];

wp(:,1:3) = wp(:,1:3)+home_pos(1:3);


figure(3); clf; hold on; grid on; axis equal

plot3(wp(:,1), wp(:,2), wp(:,3), '-', 'LineWidth', 1.5); % 'k:' makes the line black and dotted, 'LineWidth' sets the thickness

xlabel('X-axis'); % Label for the x-axis
ylabel('Y-axis'); % Label for the y-axis
zlabel('Z-axis'); % Label for the z-axis

title('3D Line Plot with Grid'); % Title for the plot

if export_traj
    save('./inference/lightbulb_trajectory.mat','wp')
    save('./inference/lightbulb_pause.mat','pause_mat')
    save('./inference/lightbulb_motor.mat','motor_mat')
end


%% Valve trajectory

export_traj = false;

tool_rot = eye(3);
tool_quat = rotm2quat(tool_rot);

n_points = 100;

theta = deg2rad(45);

Ry_mid = [cos(theta)  0  sin(theta);
          0           1  0;
          -sin(theta) 0  cos(theta)];
mid_quat = rotm2quat(Ry_mid);

theta = deg2rad(80);
Ry_end = [cos(theta)  0  sin(theta);
          0           1  0;
          -sin(theta) 0  cos(theta)];
end_quat = rotm2quat(Ry_end);


% Constant offsets based Optitrack markers used to measure positions
gripper_offset = -[0,0,17/2 + (12/2)];
valve_offset = -[(9.5/2) + 10 + (12/2),5,-15];

% Load in measured gripper position
gripper_pos = load('./state/valve_gripper.mat').pos;
p_gripper = gripper_pos(1:3) + gripper_offset;

% Load in measured
valve_pos = load('./state/valve.mat').pos;
p_valve = valve_pos(1:3) + valve_offset;

% Adjust positions based on measured distance
delta_pos = home_pos(1:3)-p_gripper;
delta_pos_rot = Ry_end*delta_pos.';
delta_pos_rot = delta_pos_rot.'; 
p_valve = p_valve + delta_pos_rot;
p_valve = p_valve-home_pos(1:3);

wp_1 = [0,0,0;
        -200,0,100;
        p_valve];

n_1 = size(wp_1,1);
wp_1 = [wp_1,[tool_quat;mid_quat;end_quat]];
wp_1 = interp_waypoints(wp_1,n_points,"cubic");
% Define pause and motor mats
pause_mat_1 = zeros([1,size(wp_1,1)]);
motor_mat_1 = zeros([1,size(wp_1,1)]);
pause_mat_1(end) = 1;
motor_mat_1(end) = 5.6;


wp = [wp_1];
pause_mat = [pause_mat_1];
motor_mat = [motor_mat_1];

wp(:,1:3) = wp(:,1:3)+home_pos(1:3);

figure(3); clf; hold on; grid on; axis equal

plot3(wp(:,1), wp(:,2), wp(:,3), '-', 'LineWidth', 1.5); % 'k:' makes the line black and dotted, 'LineWidth' sets the thickness

xlabel('X-axis'); % Label for the x-axis
ylabel('Y-axis'); % Label for the y-axis
zlabel('Z-axis'); % Label for the z-axis

title('3D Line Plot with Grid'); % Title for the plot

if export_traj
    save('./inference/valve_trajectory.mat','wp')
    save('./inference/valve_pause.mat','pause_mat')
    save('./inference/valve_motor.mat','motor_mat')
end

plot_triad(wp(:,1),wp(:,2),wp(:,3),[wp(:,7),wp(:,4),wp(:,5),wp(:,6)])

%% Helper functions

function interpolatedWaypoints = interp_waypoints(waypoints, totalPoints, mode)
    % Extract positions and quaternions
    positions = waypoints(:, 1:3);
    quaternions = waypoints(:, 4:7);

    % Total number of waypoints
    numWaypoints = size(waypoints, 1);

    % Calculate cumulative distances
    cumulativeDistances = zeros(numWaypoints, 1);
    for i = 2:numWaypoints
        cumulativeDistances(i) = cumulativeDistances(i-1) + norm(positions(i, :) - positions(i-1, :));
    end

    % Normalize cumulative distances to [0, 1] for interpolation
    normalizedCumulativeDistances = cumulativeDistances / cumulativeDistances(end);

    % Interpolate positions using cubic spline
    tPositions = linspace(0, 1, totalPoints);
    if mode == "cubic"
        interpolatedPositions = spline(normalizedCumulativeDistances, positions', tPositions)';

    elseif mode == "linear"
    interpolatedPositions = interp1(normalizedCumulativeDistances, positions, tPositions, 'linear');
    end

    % Initialize quaternion interpolation
    interpolatedQuaternions = zeros(totalPoints, 4);

    % Interpolate quaternions
    for i = 1:totalPoints
        % Find the two original waypoints surrounding the current interpolated point
        t = tPositions(i);
        idx = find(normalizedCumulativeDistances <= t, 1, 'last');
        if idx == numWaypoints
            idx = numWaypoints - 1;
        end
        nextIdx = idx + 1;

        % Calculate the fraction between the two waypoints for SLERP
        tFraction = (t - normalizedCumulativeDistances(idx)) / (normalizedCumulativeDistances(nextIdx) - normalizedCumulativeDistances(idx));

        % Perform SLERP
        qStart = quaternions(idx, :) / norm(quaternions(idx, :));
        qEnd = quaternions(nextIdx, :) / norm(quaternions(nextIdx, :));
        interpolatedQuaternions(i, :) = quatinterp(qStart, qEnd, tFraction, 'slerp');
    end

    % Combine interpolated positions and quaternions
    interpolatedWaypoints = [interpolatedPositions, interpolatedQuaternions(:,[2,3,4,1])];
end

function plot_triad(x, y, z, q)
    figure(); clf; hold on; grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    
    arrowLength = 20; % Adjust the arrow length as needed
    headSize = 20;
        

    for i = 1:length(x)
        % Convert quaternion to rotation matrix
        R = quat2rotm(q(i,:));
        
        % Origin for the triad
        origin = [x(i), y(i), z(i)];
        
        % Directions for the triad arrows, transformed by R
        xDir = R(:,1)';
        yDir = R(:,2)';
        zDir = R(:,3)';
        
        % Plot arrows
        quiver3(origin(1), origin(2), origin(3), xDir(1), xDir(2), xDir(3), arrowLength, 'r', 'LineWidth', 2, 'MaxHeadSize', headSize);
        quiver3(origin(1), origin(2), origin(3), yDir(1), yDir(2), yDir(3), arrowLength, 'g', 'LineWidth', 2, 'MaxHeadSize', headSize);
        quiver3(origin(1), origin(2), origin(3), zDir(1), zDir(2), zDir(3), arrowLength, 'b', 'LineWidth', 2, 'MaxHeadSize', headSize);
    end
    
    hold off;
end