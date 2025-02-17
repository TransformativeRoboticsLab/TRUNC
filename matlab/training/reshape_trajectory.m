close all; clc; clear all

p = load('./data/2024_02_04_11_40_02/training_waypoints.mat').points;

block_length = size(p,1);
p_reshape = zeros(size(p,1)*size(p,3),9);

for block_num = 1:size(p,3)
    block_idx_start = (block_num - 1) * block_length + 1;
    block_idx_end = block_num*block_length;
    p_reshape(block_idx_start:block_idx_end,:) = p(:,:,block_num);
end

points = p_reshape;
save('./data/2024_02_04_11_40_02/training_waypoints_matrix.mat','points');