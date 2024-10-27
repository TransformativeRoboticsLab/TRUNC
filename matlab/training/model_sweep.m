close all; clc; clear all;
addpath('./util')

arm = robotArm();

save_output = false;

%% Find lengths from model

arm_model = trunc_model();

arm_model.update_config([50,0,...
                         50,0,...
                         0,0],0);

arm_model.draw_arm()
lengths = arm_model.find_lengths()

%% Set arm pose

comp = load('./state/comp.mat').comp;
arm.set_pos(comp+2.*lengths)

%% Come up with our own compensation

% Extract each joint
r_w = 0
alpha_w = 0.1;
r_u = -60
alpha_u = 0.6;
dl_wrist = [0,r_w-20,r_w-5];
dl_elbow = [r_u,0,0];
dl_shoulder = [r_u,0,0];
dl = 0;

dl_offset = (max(abs(dl_wrist)) + max(abs(dl_elbow)) + max(abs(dl_shoulder)))/7
dl = dl + dl_offset;

% Summing segments into lengths
dl_elbow = dl_elbow + 0.75.*dl_shoulder;
dl_wrist = dl_wrist + dl_elbow;

% Add in compression values
dl_wrist = dl_wrist + dl;
dl_elbow = dl_elbow + (5/7).*dl;
dl_shoulder = dl_shoulder + (3/7).*dl;

dl_wrist = dl_wrist + alpha_w.*[r_w,0,0] + alpha_u.*[0,r_u,r_u];
dl_elbow = dl_elbow + alpha_u.*[0,r_u,r_u] ;
dl_shoulder = dl_shoulder + alpha_u.*[0,r_u,r_u];

delta_lengths = [dl_wrist(1),dl_elbow(1),dl_shoulder(1),...
                dl_wrist(2),dl_elbow(2),dl_shoulder(2),...
                dl_wrist(3),dl_elbow(3),dl_shoulder(3)]



%% Set arm pose

comp = load('./state/comp.mat').comp;
arm.set_pos(comp+delta_lengths)