close all; clc; clear all;

rng(11);
dr_max = 60; % elbow and shoulder
dr_max_wrist = 80; % wrist rotation limit
dl_max = 60; % limit for extension
n = 10000;

num_trajectories = 400;
num_waypoints = 16;
max_trajectory_length = 100;
point_per_distance = 1/10;

%% Define sweep over the configuration space

dl = -dl_max.*rand(n,1);

% Wrist
dl_wrist = -dr_max_wrist.*rand(n, 3);
for i = 1:n
    zeroIndex = randi(3);
    dl_wrist(i, zeroIndex) = 0; % Randomly zero out to prevent a length change
end

% Elbow
dl_elbow = -dr_max.*rand(n, 3);
for i = 1:n
    zeroIndex = randi(3);
    dl_elbow(i, zeroIndex) = 0; % Randomly zero out to prevent a length change
end

% Shoulder and elbow are coupled
dl_shoulder = dl_elbow;

dl_offset = (max(abs(dl_wrist)) + max(abs(dl_elbow)) + max(abs(dl_shoulder)))/8;
dl = dl + dl_offset;

% Anti-slackening compensation
dl_elbow = dl_elbow + dl_shoulder;
dl_wrist = dl_wrist + 0.75.*dl_elbow;

% Add in compression values
dl_wrist = dl_wrist + dl;
dl_elbow = dl_elbow + (5/7).*dl;
dl_shoulder = dl_shoulder + (3/7).*dl;

lengths_sweep = [dl_wrist(:,1),dl_elbow(:,1),dl_shoulder(:,1),...
                dl_wrist(:,2),dl_elbow(:,2),dl_shoulder(:,2),...
                dl_wrist(:,3),dl_elbow(:,3),dl_shoulder(:,3)];

%% Generate a matrix of trajectories

training_trajectories = zeros(max_trajectory_length,9,num_trajectories);

for t_idx = 1:num_trajectories
    % Randomly sample based on num_waypoints
    sampled_indices = randperm(n, num_waypoints);
    waypoints = lengths_sweep(sampled_indices, :);
    waypoints = [zeros(1,9);waypoints];

    % Sort waypoints using greedy_tsp
    sorted_waypoints = greedy_TSP(waypoints);

    % Linearly interpolate between waypoints based on point_per_distance
    trajectory = [];
    for wp_idx = 1:num_waypoints-1
        start_point = sorted_waypoints(wp_idx, :);
        end_point = sorted_waypoints(wp_idx + 1, :);
        distance = norm(end_point - start_point);
        num_points = round(distance * point_per_distance);
        interpolated_points = interp1([1, 2], [start_point; end_point], 1:1/num_points:2, 'linear');
        trajectory = [trajectory; interpolated_points(1:end-1, :)];                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
    end
    trajectory = [trajectory; sorted_waypoints(end, :)]; % Add the last waypoint

    % Trim based on max_trajectory length
    if size(trajectory, 1) > max_trajectory_length
        trajectory = trajectory(1:max_trajectory_length, :);
    end

    % Save to training_trajectories
    training_trajectories(:, :, t_idx) = trajectory;
end

points = training_trajectories;
save("./trajectory/training_waypoints","points")
%% Greedy solution to TSP

function delta_fast = greedy_TSP(lengths_sweep)

    delta_fast = zeros(size(lengths_sweep));
    delta_fast(1,:) = lengths_sweep(1,:);
    
    % Keep track of which answers to exclude
    exclude = true(size(delta_fast,1),1);
    exclude(1) = false;
    
    for i = 1:size(delta_fast,1)-1
        % Keep track of original indices
        original_idx = find(exclude);
        
        delta_trim = lengths_sweep(exclude,:);
        idx = knnsearch(delta_trim,delta_fast(i,:));
        
        % Update indices
        original_idx_selected = original_idx(idx);
    
        delta_fast(i+1,:) = lengths_sweep(original_idx_selected,:);
        exclude(original_idx_selected) = false;
    end
end